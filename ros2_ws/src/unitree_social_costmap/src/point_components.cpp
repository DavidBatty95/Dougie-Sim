// Exact adaptive-radius cloud connectivity, callable through a small C ABI.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace {
std::uint64_t key(int x, int y) {
  return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(y);
}
}

extern "C" int unitree_point_components(
  const double *points, const double *radii, int count,
  double grid_size, double z_gate, std::int32_t *labels)
{
  if (!points || !radii || !labels || count < 0 || !std::isfinite(grid_size) || grid_size <= 0 || !std::isfinite(z_gate)) {
    return -1;
  }
  try {
    for (int i=0; i<count; ++i) {
      if (!std::isfinite(radii[i]) || radii[i] < 0) {return -1;}
      grid_size = std::max(grid_size, radii[i]);
      for (int j=0; j<3; ++j) {
        if (!std::isfinite(points[3*i+j]) || std::abs(points[3*i+j]) > 1e6) {return -1;}
      }
    }
    grid_size = std::max(grid_size, .001);
    std::unordered_map<std::uint64_t, std::vector<int>> grid;
    std::vector<int> cell_x(count), cell_y(count);
    for (int i=0; i<count; ++i) {
      cell_x[i] = static_cast<int>(std::floor(points[3*i]/grid_size));
      cell_y[i] = static_cast<int>(std::floor(points[3*i+1]/grid_size));
      grid[key(cell_x[i],cell_y[i])].push_back(i);
      labels[i] = -1;
    }
    int component = 0;
    std::deque<int> queue;
    for (int start=0; start<count; ++start) {
      if (labels[start] >= 0) {continue;}
      labels[start] = component;
      queue.push_back(start);
      while (!queue.empty()) {
        const int i=queue.front(); queue.pop_front();
        for (int dx=-1; dx<=1; ++dx) {
          for (int dy=-1; dy<=1; ++dy) {
            auto found=grid.find(key(cell_x[i]+dx,cell_y[i]+dy));
            if (found==grid.end()) {continue;}
            for (const int j : found->second) {
              if (labels[j]>=0 || (z_gate>0 && std::abs(points[3*j+2]-points[3*i+2])>z_gate)) {continue;}
              const double x=points[3*j]-points[3*i], y=points[3*j+1]-points[3*i+1];
              const double radius=std::max(radii[i],radii[j]);
              if (x*x+y*y <= radius*radius) {
                labels[j]=component;
                queue.push_back(j);
              }
            }
          }
        }
      }
      ++component;
    }
    return component;
  } catch (...) {
    return -1;
  }
}
