"""Blocking workflows never consume the executor needed for their progress."""

from collections import deque
from threading import Event, RLock
from time import monotonic, sleep

import pytest
from rclpy.task import Task

from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.workflow_worker import run_in_worker


class CallbackQueue:
    """Exercise real ROS Tasks/Futures on one slot without starting ROS."""

    def __init__(self):
        self.pending = deque()
        self.lock = RLock()

    def create_task(self, handler, *args, **kwargs):
        task = Task(handler, args=args, kwargs=kwargs, executor=self)
        self._call_task_in_next_spin(task)
        return task

    def _call_task_in_next_spin(self, task):
        with self.lock:
            self.pending.append(task)

    def spin_once(self):
        with self.lock:
            task = self.pending.popleft() if self.pending else None
        if task is not None:
            task()


def finish(task, executor):
    deadline = monotonic() + 2.0
    while not task.done() and monotonic() < deadline:
        executor.spin_once()
        sleep(0.001)
    assert task.done()
    return task.result()


def test_one_executor_slot_can_supply_progress_for_full_worker_capacity():
    worker = OperationAdmission(1)
    started, progress = Event(), Event()
    assert worker.reserve('capture', context_id='context', lane='probe')

    def blocking(cancelled):
        started.set()
        assert progress.wait(2.0)
        assert not cancelled()
        return 'saved'

    terminal = []

    async def execute():
        result = await run_in_worker(worker, 'capture', blocking)
        terminal.append(result)
        return result

    # These Task calls represent one executor slot, including the callback the
    # blocking workflow needs. No ROS node or background executor is started.
    callbacks = CallbackQueue()
    task = callbacks.create_task(execute)
    callbacks.spin_once()
    assert started.wait(1.0)
    assert not task.done()
    assert not worker.reserve('second', context_id='other', lane='base')
    callbacks.create_task(progress.set)
    assert finish(task, callbacks) == 'saved'
    task()
    assert terminal == ['saved']
    worker.close()


def test_cancellation_retains_capacity_until_worker_exits_and_close_is_retryable():
    worker = OperationAdmission(1)
    started, observed, release = Event(), Event(), Event()
    assert worker.reserve('request', context_id='context', lane='probe')

    def blocking(cancelled):
        started.set()
        deadline = monotonic() + 2.0
        while not cancelled() and monotonic() < deadline:
            sleep(0.001)
        assert cancelled()
        observed.set()
        assert release.wait(2.0)
        return 'cancelled'

    future = run_in_worker(worker, 'request', blocking)
    assert started.wait(1.0)
    worker.cancel_context('context')
    assert observed.wait(1.0)
    worker.release_pending('request')
    assert not worker.reserve('new', context_id='different', lane='base')
    assert not worker.shutdown_status()[0]
    worker.close()
    assert not worker.shutdown_status()[0]
    release.set()

    async def result():
        return await future

    callbacks = CallbackQueue()
    assert finish(callbacks.create_task(result), callbacks) == 'cancelled'
    assert worker.shutdown_status()[0]
    worker.close()
    worker.close()
    assert not worker.reserve('late', context_id='different', lane='base')


def test_context_and_lane_are_exclusive_even_with_spare_capacity():
    worker = OperationAdmission(2)
    assert worker.reserve('one', context_id='context1', lane='probe')
    assert worker.reserve('one', context_id='context1', lane='probe')
    assert not worker.reserve('one', context_id='wrong-context', lane='probe')
    assert not worker.reserve('two', context_id='context1', lane='base')
    assert not worker.reserve('two', context_id='context2', lane='probe')
    assert worker.reserve('two', context_id='context2', lane='base')
    assert not worker.reserve('three', context_id='context3', lane='other')
    worker.close()
    assert not worker.shutdown_status()[0]
    worker.release_pending('one')
    assert not worker.shutdown_status()[0]
    worker.release_pending('two')
    assert worker.shutdown_status()[0]
    worker.close()


def test_worker_exception_has_one_terminal_result_and_releases_capacity():
    worker = OperationAdmission(1)
    assert worker.reserve('request', context_id='context', lane='probe')

    def fail(_cancelled):
        raise ValueError('save failed')

    async def execute():
        return await run_in_worker(worker, 'request', fail)

    with pytest.raises(ValueError, match='save failed'):
        callbacks = CallbackQueue()
        finish(callbacks.create_task(execute), callbacks)
    assert worker.reserve('retry', context_id='context', lane='probe')
    worker.cancel_context('context')
    worker.close()


@pytest.mark.parametrize('limit', [0, -1, True, 1.5])
def test_worker_limit_must_be_a_positive_integer(limit):
    with pytest.raises(ValueError):
        OperationAdmission(limit)


def test_admission_releases_exact_request_and_closes_idempotently():
    admission = OperationAdmission(1)
    assert admission.reserve('first', context_id='context', lane='arm')
    admission.release('foreign')
    assert not admission.reserve('second', context_id='context', lane='arm')
    admission.release('first')
    assert admission.reserve('second', context_id='context', lane='arm')
    admission.close()
    admission.release('second')
    admission.close()
    assert not admission.reserve('third')


def test_context_close_releases_disconnected_goal_but_retains_active_work():
    admission = OperationAdmission(1)
    worker = admission
    assert worker.reserve('disconnected', context_id='context', lane='probe')
    assert not admission.reserve('motion', context_id='other', lane='physical')
    admission.cancel_context('context')
    assert worker.shutdown_status()[0]
    assert admission.reserve('motion', context_id='other', lane='physical')
    assert admission.claim('motion')
    admission.cancel_context('other')
    assert admission.cancel_requested('motion')
    assert not worker.reserve('capture', context_id='context', lane='probe')
    admission.release('motion')
    assert worker.reserve('capture', context_id='context', lane='probe')
    worker.cancel_context('context')
    worker.close()


def test_shutdown_retains_accepted_goals_until_their_queued_callbacks_terminate():
    admission = OperationAdmission(1)
    worker = admission
    assert worker.reserve('accepted', context_id='context', lane='probe')
    callbacks = CallbackQueue()
    terminal = []
    started = Event()

    async def execute():
        try:
            return await run_in_worker(worker, 'accepted', lambda _cancelled: started.set())
        except RuntimeError:
            terminal.append('aborted')
            return 'aborted'
        finally:
            worker.release_pending('accepted')

    queued = callbacks.create_task(execute)
    admission.close()
    worker.close()
    worker.close()
    assert not worker.shutdown_status()[0]
    confirmed, detail = admission.shutdown_status()
    assert not confirmed
    assert '1 queued' in detail
    worker.close()
    assert not worker.shutdown_status()[0]
    assert finish(queued, callbacks) == 'aborted'
    assert terminal == ['aborted']
    assert not started.is_set()
    assert worker.shutdown_status()[0]
    assert admission.shutdown_status()[0]
    worker.close()


def test_shutdown_does_not_discard_claimed_ownership_when_releasing_a_queued_goal():
    admission = OperationAdmission(2)
    assert admission.reserve('queued', context_id='queued-context', lane='arm')
    assert admission.reserve('active', context_id='active-context', lane='base')
    assert admission.claim('active')
    admission.close()
    admission.release_pending('active')
    assert not admission.shutdown_status()[0]
    admission.release_pending('queued')
    confirmed, detail = admission.shutdown_status()
    assert not confirmed
    assert '0 queued, 1 executing' in detail
    admission.release('active')
    assert admission.shutdown_status()[0]
