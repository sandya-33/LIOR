#include "filter.h"
namespace pcl {

    template <typename PointT>
    void CustomFilter<PointT>::applyFilter(PointCloud& output) {
        output = *input_;
    }
}