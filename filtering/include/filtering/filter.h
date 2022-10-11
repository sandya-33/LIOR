#pragma once

#include <cmath>
#include <vector>
#include <omp.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/organized.h>


namespace pcl {
	template<typename PointT>
	class CustomFilter : public Filter<PointT> {
		using Filter<PointT>::input_;
		using Filter<PointT>::indices_;
		typedef typename Filter<PointT>::PointCloud PointCloud;
		typedef typename pcl::KdTreeFLANN<PointT> KdTree;
	private:
		/** \brief Private variable details */
		int private_variable;
		
		/** \brief A pointer to the spatial search object, KdTreeFLANN for fast search methods. */
		KdTree kdTree_;

		/** \brief Exaple of Inline function that calculates distance
		* \param[in] point current point of interest
		*/
		inline double computeDistance(auto point){
			return (sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z)));
		}
		

	public:
		/** \brief Constructor.
		 * Sets \ref angle_resol_ to 0 and \ref multi_ to
		 * Sets angle_resol_ to 0 and \ref multi_ to MAXDBL
		 */
		CustomFilter() {
			private_variable = -1;
		}

		/** \brief Set the private variable
		* \param[in] private_var private_var value
		*/
		inline void setPrivateVariable(int private_var) {
			private_variable = private_var;
		}

		/** \brief Get the private variable value as set by the user. */\
			inline int getPrivateVariable() const {
			return(private_variable);
		}

		/** \brief Provide a pointer to the search object.
		 * \param[in] tree a pointer to the spatial search object.
		 */
		void setSearchMethod(const KdTree& kdtree) {
			kdTree_ = kdtree;
		}

		/** \brief pre condition checking before filtering */
		bool inline preCheck() {

			if (private_variable == -1) {
				PCL_ERROR("[pcl::CustomFilter] Private Variable cannot be -1.\n");
				return false;
			}

			return true;
		}

		/** \brief Filter the input data and store the results into output. Default Filter
		 * \param[out] output the resultant point cloud message
		 */
		void applyFilter(PointCloud& output);

	};
}

