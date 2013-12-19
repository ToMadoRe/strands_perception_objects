#ifndef MYGRAPHCLASSES_H
#define MYGRAPHCLASSES_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <faat_pcl/3d_rec_framework/feature_wrapper/local/image/sift_local_estimator.h>
#include <faat_pcl/3d_rec_framework/defines/faat_3d_rec_framework_defines.h>
#include <faat_pcl/recognition/hv/hv_go_3D.h>
#include <faat_pcl/3d_rec_framework/pc_source/model_only_source.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
//#include <pcl/common/common.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <faat_pcl/registration/fast_icp_with_gc.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions.h>
// #include "functions.h"
//#include "visual_recognizer/Hypotheses.h"
#include "recognition_service/recognize.h"
#include "scitos_apps_msgs/action_buttons.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointInT;
typedef PointInT::ConstPtr ConstPointInTPtr;
typedef boost::shared_ptr< PointInT > PointInTPtr;
typedef pcl::PointXYZRGB PointT;
typedef pcl::Histogram<128> FeatureT;
typedef flann::L1<float> DistT;
class Hypothesis
{
public:
    Hypothesis ( std::string model_id, Eigen::Matrix4f transform, std::string origin, bool extended = false, bool verified = false );
    std::string model_id_, origin_;
    Eigen::Matrix4f transform_;
    bool extended_;
    bool verified_;
};

class View
{
public:
    View();
    //View(const View &view);
    int foo;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> > pSceneXYZRGBNormal;
    boost::shared_ptr< pcl::PointCloud<pcl::Normal> > pSceneNormals;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl_f; //no table plane
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pScenePCl_f_ds;
    boost::shared_ptr< pcl::PointCloud<FeatureT > > pSignatures;
    boost::shared_ptr< pcl::PointIndices > pIndices_above_plane;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pKeypoints;
    std::string scene_filename;
    std::vector<std::string> model_ids;
    std::vector<double> modelToViewCost;
    std::vector<Hypothesis> hypothesis;
    std::vector<Hypothesis> hypothesis_single_unverified;
    Eigen::Matrix4f absolute_pose;
};

class myEdge
{
public:
    Eigen::Matrix4f transformation;
    double edge_weight;
    std::string model_name;
    int source_id, target_id;
    std::vector <cv::DMatch> matches;
};

using namespace boost;
typedef adjacency_list < vecS, vecS, undirectedS, property<vertex_distance_t, int>, property<edge_weight_t, double> > GraphMST;
typedef graph_traits < GraphMST >::vertex_descriptor VertexMST;
typedef graph_traits < GraphMST >::edge_descriptor EdgeMST;

//--"copy"-of-graph-to-save-custom-information------prim_minimum_spanning_tree----cannot(?)-handle-internal-bundled-properties---
typedef adjacency_list < vecS, vecS, undirectedS, View, myEdge > Graph;
typedef graph_traits < Graph >::vertex_descriptor Vertex;
typedef graph_traits < Graph >::edge_descriptor Edge;
typedef graph_traits<Graph>::vertex_iterator vertex_iter;
typedef property_map<Graph, vertex_index_t>::type IndexMap;

typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
typedef boost::shared_ptr<ModelT> ModelTPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;
std::vector<Vertex> my_node_reader ( std::string filename, Graph &g );
void createBigPointCloud ( Graph & grph_final,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud );

bool calcFeatures ( Vertex &src, Graph &grph );
void estimateViewTransformationBySIFT ( const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT > *flann_index, Eigen::Matrix4f &transformation, Edge &edge );
void selectLowestWeightEdgesFromParallelEdges ( const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges );
void extendHypothesis ( Graph &grph );
void calcMST ( const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final );
void createEdgesFromHypothesisMatch ( const std::vector<Vertex> &vertices_v, Graph &grph, std::vector<Edge> &edges );
void calcEdgeWeight ( std::vector<Edge> &edges, Graph &grph, bool edge_weight_by_scene=true, bool edge_weight_by_projection=true );
double calcRegistrationCost ( pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr pOctree, int K=1, double beta = 0.2 );
double calcRegistrationCost ( pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl, pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr pOrganizedNeighbor, int K=1, double beta=0.2 );
double calcRegistrationCost ( pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl,
                              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,
                              std::vector<int> & unused, double beta=0.2 );

std::vector<int> visualization_framework ( pcl::visualization::PCLVisualizer::Ptr vis, int number_of_views, int number_of_subwindows_per_view );


class multiviewGraph
{
private:
    Graph grph_, grph_final_;
    std::vector<Vertex> vertices_v_;
    std::vector<Edge> edges_;
    std::string topic_, models_dir_;
    ros::NodeHandle  *n_;
    ros::ServiceClient client_;
    ros::Subscriber sub_joy_, sub_pc_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_;
    boost::mutex current_cloud_mutex_;
    boost::shared_ptr < faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, PointT> > models_source_;
    std::string object_to_be_highlighted_;
    pcl::visualization::PCLVisualizer::Ptr vis_;
    bool visualize_output_;
    std::vector<int> viewportNr_;
    unsigned long recorded_clouds_;

public:
    multiviewGraph(){
      recorded_clouds_=0;
    }
    void joyCallback ( const scitos_apps_msgs::action_buttons& msg );
    void kinectCallback ( const sensor_msgs::PointCloud2& msg );
    int recognize ( pcl::PointCloud<pcl::PointXYZRGB> &cloud );
    void init ( int argc, char **argv );
};


namespace multiview
{
void
nearestKSearch ( flann::Index<flann::L1<float> > * index,
                 float * descr, int descr_size,
                 int k,flann::Matrix<int> &indices,flann::Matrix<float> &distances );

template <typename Type>
void
convertToFLANN ( typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data );
}

#endif
