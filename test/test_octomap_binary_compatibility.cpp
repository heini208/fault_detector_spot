// Offline proof for the installed RTAB-Map subtype, without ROS initialization.
#include <rtabmap/core/global_map/OctoMap.h>
#include <octomap/OcTree.h>

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace
{
void require(bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error(message);
}

using LeafKey = std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>;

template <typename Tree>
std::map<LeafKey, bool> leaves(const Tree& tree)
{
  std::map<LeafKey, bool> result;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    const auto& key = it.getKey();
    result.emplace(LeafKey(key[0], key[1], key[2], it.getDepth()),
                   tree.isNodeOccupied(*it));
  }
  return result;
}

void verify_roundtrip(double resolution, const char* fixture_path)
{
  rtabmap::RtabmapColorOcTree source(resolution);
  require(source.getTreeType() == "ColorOcTree", "Unexpected RTAB-Map tree type");
  require(source.getTreeDepth() == 16, "Unexpected OctoMap tree depth");

  // Eight sibling leaves of each state exercise RTAB-Map's actual pruning.
  // Different colors and RTAB-Map metadata must not affect binary occupancy.
  for (int x = 0; x < 2; ++x)
    for (int y = 0; y < 2; ++y)
      for (int z = 0; z < 2; ++z)
      {
        auto* occupied = source.updateNode(
            (10.5 + x) * resolution, (4.5 + y) * resolution,
            (8.5 + z) * resolution, true, true);
        occupied->setColor(10 + x, 40 + y, 70 + z);
        occupied->setNodeRefId(101);
        occupied->setOccupancyType(rtabmap::RtabmapColorOcTreeNode::kTypeObstacle);
        auto* free = source.updateNode(
            (-11.5 + x) * resolution, (-5.5 + y) * resolution,
            (-9.5 + z) * resolution, false, true);
        free->setColor(70 + x, 40 + y, 10 + z);
        free->setNodeRefId(102);
        free->setOccupancyType(rtabmap::RtabmapColorOcTreeNode::kTypeEmpty);
      }

  source.updateNode(-3.5 * resolution, 1.5 * resolution, 0.5 * resolution, true, true);
  source.updateNode(2.5 * resolution, -3.5 * resolution, 4.5 * resolution, false, true);
  source.updateInnerOccupancy();
  source.prune();

  bool has_pruned_leaf = false;
  bool has_occupied = false;
  bool has_free = false;
  for (auto it = source.begin_leafs(); it != source.end_leafs(); ++it)
  {
    has_pruned_leaf |= it.getDepth() < source.getTreeDepth();
    has_occupied |= source.isNodeOccupied(*it);
    has_free |= !source.isNodeOccupied(*it);
  }
  require(has_pruned_leaf && has_occupied && has_free, "Fixture lacks required states");

  std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
  source.writeBinaryData(bytes);  // Same writer used by octomap_msgs::binaryMapToMsg.
  require(!bytes.str().empty(), "Unexpected empty fixture");
  if (fixture_path)
  {
    std::ofstream fixture(fixture_path, std::ios::binary);
    fixture << bytes.str();
    require(fixture.good(), "Could not write requested binary fixture");
  }

  octomap::OcTree imported(resolution);
  imported.readBinaryData(bytes);
  require(!bytes.fail(), "OcTree rejected RTAB-Map binary occupancy");
  require(bytes.peek() == std::char_traits<char>::eof(), "Unread payload remains");
  require(imported.getResolution() == source.getResolution(), "Resolution changed");
  require(leaves(imported) == leaves(source), "Leaf key, depth, or occupancy changed");

  // Binary encoding preserves the classification, not the probability/color.
  for (auto it = source.begin_leafs(); it != source.end_leafs(); ++it)
  {
    const auto* node = imported.search(it.getCoordinate());
    require(node != nullptr, "Known leaf became unknown");
    require(imported.isNodeOccupied(node) == source.isNodeOccupied(*it),
            "Coordinate lookup changed occupancy");
  }
  for (const auto& unknown : {octomap::point3d(100, 100, 100),
                              octomap::point3d(-100, -100, -100),
                              octomap::point3d(100, -100, 100)})
  {
    require(source.search(unknown) == nullptr, "Unknown fixture point is known");
    require(imported.search(unknown) == nullptr, "Unknown point became occupied/free");
  }
  std::cout << "RTAB-Map ColorOcTree -> OcTree: resolution=" << resolution
            << " leaves=" << source.getNumLeafNodes()
            << " bytes=" << bytes.str().size() << " occupied/free/unknown/pruned preserved\n";
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    verify_roundtrip(0.04, argc > 1 ? argv[1] : nullptr);
    verify_roundtrip(0.1, nullptr);
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
