"""Basic settings editor; saving never changes live robot state."""

import math

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox, QLabel,
    QCheckBox, QComboBox, QDoubleSpinBox, QPushButton, QScrollArea,
)
from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, factory_settings, config_parameters,
)


class _NoWheelSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class _NoWheelComboBox(QComboBox):
    def wheelEvent(self, event):
        event.ignore()


class BasicSettingsPage(QWidget):
    title = 'Basic settings'
    factory_defaults = staticmethod(factory_settings)

    def __init__(self, parent=None, repository=None):
        super().__init__(parent)
        self.repository = repository if repository is not None else BasicSettingsRepository()
        self.defaults = self.factory_defaults()
        self.fields = {}
        self.factors = {}
        self._reset_pending = False
        layout = QVBoxLayout(self)
        note = QLabel(
            f'{self.title}\nSave your changes, then restart both the control window and robot application to apply them. '
            'Only edited values override the existing configuration. Saving does not change an active movement.'
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        self.groups = QVBoxLayout(content)
        scroll.setWidget(content)
        layout.addWidget(scroll)
        self._build_fields()
        self.groups.addStretch()
        buttons = QHBoxLayout()
        reset = QPushButton('Reset to defaults')
        reset.clicked.connect(self.reset_defaults)
        reload_button = QPushButton('Discard unsaved changes')
        reload_button.clicked.connect(self.reload)
        self.save_button = QPushButton('Save settings')
        self.save_button.clicked.connect(self.save)
        for button in (reset, reload_button, self.save_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.reload()
        for field in self.fields.values():
            signal = (field.toggled if isinstance(field, QCheckBox) else
                      field.currentIndexChanged if isinstance(field, QComboBox) else field.valueChanged)
            signal.connect(self._mark_dirty)

    def _build_fields(self):
        startup = self._group('Startup defaults')
        self._boolean(startup, 'mount_approval_at_startup', 'Automatically approve the mount at startup',
                      'Approves the previously selected sensor mount, or bare hand, when the robot application starts. '
                      'Assumes the physical attachment has not changed.')
        self._boolean(startup, 'environment_collision_checking', 'Check for surrounding obstacles at startup',
                      'Use MoveIt to check arm paths against surrounding obstacles after the mount is approved. '
                      'Checks for collisions with the robot itself are controlled separately.')
        modes = self._group('Default movement modes')
        for key, label in [('arm_mode', 'Default arm mode'), ('base_mode', 'Default walking mode')]:
            field = _NoWheelComboBox()
            field.addItem('Normal', 'normal')
            field.addItem('Precision', 'precision')
            field.setToolTip(
                'The mode initially selected in the movement controls. '
                + ('Precision adds checks and corrections to improve the final arm position.'
                   if key == 'arm_mode' else
                   'Precision uses a separate walking speed and gait; greater accuracy is not guaranteed.')
            )
            self.fields[key] = field
            modes.addRow(label, field)
        for part, title in [('arm', 'Arm speeds'), ('base', 'Walking speeds')]:
            form = self._group(title)
            if part == 'base':
                hint = QLabel('Defaults keep the separate speeds for distance and tag moves. '
                              'Editing a speed below sets it for both.')
                hint.setWordWrap(True)
                form.addRow(hint)
            for mode in ('normal', 'precision'):
                self._number(form, f'{part}_{mode}_linear_speed_mps', f'{mode.title()} movement speed', 1000., ' mm/s')
                self._number(form, f'{part}_{mode}_angular_speed_rad_s', f'{mode.title()} turning speed', 180. / math.pi, ' °/s')
                if part == 'base':
                    base = config_parameters('base_motion.yaml')
                    relative = base[f'base.walking.{mode}.relative_speed_mps'] * 1000.
                    tag = base[f'base.walking.{mode}.tag_speed_mps'] * 1000.
                    self.fields[f'{part}_{mode}_linear_speed_mps'].setToolTip(
                        f'Factory defaults: {relative:g} mm/s for distance moves, {tag:g} mm/s for tag moves. '
                        'Unchanged fields keep the startup configuration. Editing this value sets both speeds.'
                    )
                else:
                    self.fields[f'{part}_{mode}_linear_speed_mps'].setToolTip(
                        'How fast the hand or probe moves, in millimetres per second. '
                        + ('Unless explicitly changed, Precision uses the effective Normal speed.'
                           if mode == 'precision' else '')
                    )
                self.fields[f'{part}_{mode}_angular_speed_rad_s'].setToolTip(
                    'How fast the hand or probe changes direction, in degrees per second.' if part == 'arm' else
                    'How fast Spot turns, in degrees per second.'
                )
            if part == 'arm':
                self._number(form, 'ready_linear_speed_mps', 'Move to Ready pose speed', 1000., ' mm/s')
                self.fields['ready_linear_speed_mps'].setToolTip(
                    'How fast the arm moves into its Ready position.'
                )
        force = self._group('Force sensitivity')
        self._number(force, 'force_threshold_percent', 'Force needed to trigger a stop', 1., ' %')
        self.fields['force_threshold_percent'].setToolTip(
            '100% uses the default force thresholds. Higher values allow more force before stopping; '
            'lower values stop sooner. Must be greater than zero. '
            'The separate maximum safety force and surface-approach/contact-search thresholds stay unchanged.'
        )
        force_hint = QLabel('100% = default. Higher values allow more force before stopping.')
        force_hint.setWordWrap(True)
        force.addRow(force_hint)

    def _mark_dirty(self, *_):
        self.status.setText('Unsaved changes. Save, then restart both the control window and robot application to apply.')

    def _group(self, title):
        box = QGroupBox(title)
        form = QFormLayout(box)
        self.groups.addWidget(box)
        return form

    def _boolean(self, form, key, label, tip):
        field = QCheckBox('OFF')
        field.toggled.connect(lambda checked: field.setText('ON' if checked else 'OFF'))
        field.setToolTip(tip)
        self.fields[key] = field
        form.addRow(label, field)

    def _number(self, form, key, label, factor, suffix):
        field = _NoWheelSpinBox()
        field.setDecimals(2)
        field.setRange(0.01, 1e9)
        field.setSuffix(suffix)
        field.setKeyboardTracking(False)
        self.fields[key] = field
        self.factors[key] = factor
        form.addRow(label, field)

    def _show_values(self, values):
        self._shown_values = dict(values)
        self._shown_numbers = {}
        for key, field in self.fields.items():
            value = values[key]
            if isinstance(field, QCheckBox):
                field.setChecked(value)
            elif isinstance(field, QComboBox):
                field.setCurrentIndex(field.findData(value))
            else:
                field.setValue(value * self.factors[key])
                self._shown_numbers[key] = field.value()
        if hasattr(self, 'status'):
            self.status.setText('Changes are not saved. Click Save settings to keep them.')

    def reset_defaults(self):
        self._reset_pending = True
        self._show_values(self.defaults)
        self.status.setText('Click Save settings to remove this tab’s overrides and restore the original defaults.')

    def _display_values(self, saved):
        values = {**self.defaults, **saved}
        if 'arm_normal_linear_speed_mps' in self.defaults:
            arm = config_parameters('arm_motion.yaml')
            for kind, unit in (('linear', 'mps'), ('angular', 'rad_s')):
                key = f'arm_precision_{kind}_speed_{unit}'
                if key not in saved and arm[f'arm.motion.precision_{kind}_speed_{unit}'] == 0.0:
                    values[key] = values[f'arm_normal_{kind}_speed_{unit}']
        return values

    def reload(self):
        try:
            saved = self.repository.load()
            self._show_values(self._display_values(saved))
            self._reset_pending = False
        except (OSError, ValueError) as error:
            self.save_button.setEnabled(False)
            self.status.setText(f'Could not load settings: {error}')
            return
        self.save_button.setEnabled(True)
        self.status.setText(f'Saved settings: {self.repository.path}')

    def save(self):
        values = {}
        for key, field in self.fields.items():
            if isinstance(field, QCheckBox):
                values[key] = field.isChecked()
            elif isinstance(field, QComboBox):
                values[key] = field.currentData()
            else:
                values[key] = (self._shown_values[key] if field.value() == self._shown_numbers[key]
                               else field.value() / self.factors[key])
                if type(self.defaults[key]) is int:
                    values[key] = int(values[key])
        try:
            saved = self.repository.load()
            if self._reset_pending:
                saved = {key: value for key, value in saved.items() if key not in self.fields}
            for key, value in values.items():
                if value != self._shown_values[key]:
                    saved[key] = value
            self.repository.save(saved)
        except (OSError, ValueError) as error:
            self.status.setText(f'Could not save settings: {error}')
            return
        self.reload()
        self.status.setText('Saved. Restart both the control window and robot application to apply changes.')
