#include "myGraphClasses.h"
#include <pcl/common/transforms.h>

std::vector<Vertex>
my_node_reader (std::string filename, Graph &g)
{
  std::string fn, model_id, line, tf_str, origin, verified;
  Eigen::Matrix4f tf;
  std::ifstream myfile;
  std::vector<Vertex> vertices_temp_v;

  myfile.open (filename.c_str ());

  if (myfile.is_open ())
  {
    while (myfile.good ())
    {
      std::getline (myfile, line);

      int found = -1;
      std::string searchstring ("[file=\"");
      found = line.find (searchstring);

      if (found > -1)
      {
        Vertex v = boost::add_vertex (g);
        vertices_temp_v.push_back (v);

        fn = line.substr (found + searchstring.length ());
        fn.erase (fn.end () - 2, fn.end ());

        int read_state = 0;
        while (myfile.good () && read_state > -1)
        {
          std::getline (myfile, line);

          searchstring = ";";
          found = line.find (searchstring);
          if (found > -1)
          {
            read_state = -1;
            break;
          }
          else
          {
            searchstring = "[hypothesis_model_id=\"";
            found = line.find (searchstring);
            if (found > -1)
            {
              model_id = line.substr (found + searchstring.length ());
              model_id.erase (model_id.end () - 2, model_id.end ());
              read_state++;
            }

            searchstring = "[hypothesis_transform=\"";
            found = line.find (searchstring);
            if (found > -1)
            {
              tf_str = line.substr (found + searchstring.length ());
              tf_str.erase (tf_str.end () - 2, tf_str.end ());

              std::stringstream (tf_str) >> tf (0, 0) >> tf (0, 1) >> tf (0, 2) >> tf (0, 3) >> tf (1, 0) >> tf (1, 1) >> tf (1, 2) >> tf (1, 3)
                  >> tf (2, 0) >> tf (2, 1) >> tf (2, 2) >> tf (2, 3) >> tf (3, 0) >> tf (3, 1) >> tf (3, 2) >> tf (3, 3);
              read_state++;

            }
            searchstring = "[hypothesis_origin=\"";
            found = line.find (searchstring);
            if (found > -1)
            {
              origin = line.substr (found + searchstring.length ());
              origin.erase (origin.end () - 2, origin.end ());
              read_state++;
            
            searchstring = "[hypothesis_verified=\"";
            found = line.find (searchstring);
            if (found > -1)
            {
              verified = line.substr (found + searchstring.length ());
              verified.erase (verified.end () - 2, verified.end ());
              read_state++;
            }
            }
          }
          if (read_state >= 4)
          {
            read_state = 0;
            Hypothesis hypothesis (model_id, tf, origin, false, atoi(verified.c_str()));
            g[v].hypothesis.push_back (hypothesis);

            g[v].scene_filename = fn;
            g[v].pScenePCl.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile (g[v].scene_filename, *(g[v].pScenePCl));

          }
        }
      }
    }
    myfile.close ();
  }
  return vertices_temp_v;
}

struct my_node_writer
{
  my_node_writer (Graph& g_) :
    g (g_)
  {
  }
  ;
  template<class Vertex>
    void
    operator() (std::ostream& out, Vertex v)
    {
      out << " [label=\"" << boost::get (vertex_index, g, v) << "(" << boost::filesystem::path (g[v].scene_filename).stem ().string () << ")\"]"
          << std::endl;
      out << " [file=\"" << g[v].scene_filename << "\"]" << std::endl;
      out << " [index=\"" << boost::get (vertex_index, g, v) << "\"]" << std::endl;

      for (std::vector<Hypothesis>::iterator it_hyp = g[v].hypothesis.begin (); it_hyp != g[v].hypothesis.end (); ++it_hyp)
      {
        out << " [hypothesis_model_id=\"" << it_hyp->model_id_ << "\"]" << std::endl;
        out << " [hypothesis_transform=\"" << it_hyp->transform_ (0, 0) << " " << it_hyp->transform_ (0, 1) << " " << it_hyp->transform_ (0, 2)
            << " " << it_hyp->transform_ (0, 3) << " " << it_hyp->transform_ (1, 0) << " " << it_hyp->transform_ (1, 1) << " "
            << it_hyp->transform_ (1, 2) << " " << it_hyp->transform_ (1, 3) << " " << it_hyp->transform_ (2, 0) << " " << it_hyp->transform_ (2, 1)
            << " " << it_hyp->transform_ (2, 2) << " " << it_hyp->transform_ (2, 3) << " " << it_hyp->transform_ (3, 0) << " "
            << it_hyp->transform_ (3, 1) << " " << it_hyp->transform_ (3, 2) << " " << it_hyp->transform_ (3, 3) << " " << "\"]" << std::endl;
        out << " [hypothesis_origin=\"" << it_hyp->origin_ << "\"]" << std::endl;
        out << " [hypothesis_verified=\"" << it_hyp->verified_ << "\"]" << std::endl;
      }
    }
  ;
  Graph g;
};

struct my_edge_writer
{
  my_edge_writer (Graph& g_) :
    g (g_)
  {
  }
  ;
  template<class Edge>
    void
    operator() (std::ostream& out, Edge e)
    {
      // just an example, showing that local options override global
      out << " [color=purple]" << std::endl;
      out << " [label=\"" << g[e].edge_weight << boost::filesystem::path (g[e].model_name).stem ().string () << "\"]" << std::endl;
    }
  ;
  Graph g;
};

struct my_graph_writer
{
  void
  operator() (std::ostream& out) const
  {
    out << "node [shape=circle color=blue]" << std::endl;
    // just an example, showing that local options override global
    out << "edge [color=red]" << std::endl;
  }
} myGraphWrite;

void
outputgraph (Graph& map, const char* filename)
{
  std::ofstream gout;
  gout.open (filename);
  write_graphviz (gout, map, my_node_writer (map), my_edge_writer (map), myGraphWrite);
}

inline void
createBigPointCloudRecursive (Graph & grph_final, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud, Vertex start, Vertex coming_from,
                              Eigen::Matrix4f accum)
{
  if (boost::degree (start, grph_final) == 1)
  {
    //check if target is like coming_from
    boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
    for (boost::tie (ei, ei_end) = boost::out_edges (start, grph_final); ei != ei_end; ++ei)
    {
      if (target (*ei, grph_final) == coming_from)
        return;
    }
  }

  boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
  std::vector < boost::graph_traits<Graph>::out_edge_iterator > edges;
  for (boost::tie (ei, ei_end) = boost::out_edges (start, grph_final); ei != ei_end; ++ei)
  {

    if (target (*ei, grph_final) == coming_from)
    {
      continue;
    }

    edges.push_back (ei);
  }

  for (size_t i = 0; i < edges.size (); i++)
  {
    Eigen::Matrix4f internal_accum;
    Edge e = *edges[i];
    Vertex src = boost::source (e, grph_final);
    Vertex targ = boost::target (e, grph_final);
    Eigen::Matrix4f transform;
    if (grph_final[e].source_id == boost::get (vertex_index, grph_final, src))
    {
      PCL_WARN("inverse");
      transform = grph_final[e].transformation.inverse ();
    }
    else
    {
      PCL_WARN("normal");
      transform = grph_final[e].transformation;
    }

    internal_accum = accum * transform;
    std::cout << internal_accum << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*grph_final[targ].pScenePCl, *trans, internal_accum);
    *big_cloud += *trans;
    grph_final[targ].absolute_pose = internal_accum;
    createBigPointCloudRecursive (grph_final, big_cloud, targ, start, internal_accum);
  }
}

// View::View(const View &view)
// {
//     pScenePCl = view.pScenePCl;
//     pScenePCl_f = view.pScenePCl_f;
//     pScenePCl_f_ds = view.pScenePCl_f_ds;
//     pSceneNormal = view.pSceneNormal;
//     scene_filename = view.scene_filename;
//     hypothesis = view.hypothesis;
// }


void
createBigPointCloud (Graph & grph_final, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & big_cloud)
{
  std::pair<vertex_iter, vertex_iter> vp;
  vp = vertices (grph_final);
  Eigen::Matrix4f accum;
  accum.setIdentity ();
  *big_cloud += *grph_final[*vp.first].pScenePCl;
  grph_final[*vp.first].absolute_pose = accum;
  createBigPointCloudRecursive (grph_final, big_cloud, *vp.first, *vp.first, accum);
}

bool
calcFeatures (Vertex &src, Graph &grph)
{
  boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT> > estimator;
  estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, FeatureT>);

  PointInTPtr processed (new PointInT);
  estimator->setIndices (*(grph[src].pIndices_above_plane));
  return estimator->estimate (grph[src].pScenePCl, processed, grph[src].pKeypoints, grph[src].pSignatures);

  //----display-keypoints--------------------
  /*pcl::visualization::PCLVisualizer::Ptr vis_temp (new pcl::visualization::PCLVisualizer);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[*it_vrtx].pScenePCl);
   vis_temp->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb_verified, "Hypothesis_1");
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (grph[*it_vrtx].pKeypoints);

   for (size_t keyId = 0; keyId < grph[*it_vrtx].pKeypoints->size (); keyId++)
   {
   std::stringstream sphere_name;
   sphere_name << "sphere_" << keyId;
   vis_temp->addSphere<pcl::PointXYZRGB> (grph[*it_vrtx].pKeypoints->at (keyId), 0.01, sphere_name.str ());
   }
   vis_temp->spin ();*/
}

void
multiview::nearestKSearch (flann::Index<flann::L1<float> > * index, float * descr, int descr_size, int k, flann::Matrix<int> &indices,
                           flann::Matrix<float> &distances)
{
  flann::Matrix<float> p = flann::Matrix<float> (new float[descr_size], 1, descr_size);
  memcpy (&p.ptr ()[0], &descr[0], p.cols * p.rows * sizeof(float));

  index->knnSearch (p, indices, distances, k, flann::SearchParams (128));
  delete[] p.ptr ();
}

template<typename Type>
  void
  multiview::convertToFLANN (typename pcl::PointCloud<Type>::Ptr & cloud, flann::Matrix<float> &data)
  {
    data.rows = cloud->points.size ();
    data.cols = sizeof(cloud->points[0].histogram) / sizeof(float); // number of histogram bins

    std::cout << data.rows << " " << data.cols << std::endl;

    flann::Matrix<float> flann_data (new float[data.rows * data.cols], data.rows, data.cols);

    for (size_t i = 0; i < data.rows; ++i)
      for (size_t j = 0; j < data.cols; ++j)
      {
        flann_data.ptr ()[i * data.cols + j] = cloud->points[i].histogram[j];
      }

    data = flann_data;
  }

template void
multiview::convertToFLANN<pcl::Histogram<128> > (pcl::PointCloud<pcl::Histogram<128> >::Ptr & cloud, flann::Matrix<float> &data); // explicit instantiation.


void
estimateViewTransformationBySIFT (const Vertex &src, const Vertex &trgt, Graph &grph, flann::Index<DistT> *flann_index, Eigen::Matrix4f &transformation,
                                  Edge &edge)
{
  int K = 1;
  flann::Matrix<int> indices = flann::Matrix<int> (new int[K], 1, K);
  flann::Matrix<float> distances = flann::Matrix<float> (new float[K], 1, K);

  pcl::CorrespondencesPtr temp_correspondences (new pcl::Correspondences);
  for (size_t keypointId = 0; keypointId < grph[src].pKeypoints->size (); keypointId++)
  {
    FeatureT searchFeature = grph[src].pSignatures->at (keypointId);
    int size_feat = sizeof(searchFeature.histogram) / sizeof(float);
    multiview::nearestKSearch (flann_index, searchFeature.histogram, size_feat, K, indices, distances);

    pcl::Correspondence corr;
    corr.distance = distances[0][0];
    corr.index_query = keypointId;
    corr.index_match = indices[0][0];
    temp_correspondences->push_back (corr);
  }
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej;
  rej.reset (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ());
  pcl::CorrespondencesPtr after_rej_correspondences (new pcl::Correspondences ());

  rej->setMaximumIterations (50000);
  rej->setInlierThreshold (0.02);
  rej->setInputTarget (grph[trgt].pKeypoints);
  rej->setInputSource (grph[src].pKeypoints);
  rej->setInputCorrespondences (temp_correspondences);
  rej->getCorrespondences (*after_rej_correspondences);

  transformation = rej->getBestTransformation ();
  pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;
  t_est.estimateRigidTransformation (*grph[src].pKeypoints, *grph[trgt].pKeypoints, *after_rej_correspondences, transformation);

  std::cout << "size of corr before " << temp_correspondences->size () << "; after: " << after_rej_correspondences->size () << std::endl;

  bool b;
  tie (edge, b) = add_edge (trgt, src, grph);
  grph[edge].transformation = transformation.inverse ();
  grph[edge].model_name = std::string ("scene_to_scene");
  grph[edge].source_id = boost::get (vertex_index, grph, trgt);
  grph[edge].target_id = boost::get (vertex_index, grph, src);

  /*
   pcl::visualization::PCLVisualizer::Ptr vis_temp2 (new pcl::visualization::PCLVisualizer);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (grph[*it_vrtx].pScenePCl);
   vis_temp2->addPointCloud<pcl::PointXYZRGB> (grph[*it_vrtx].pScenePCl, handler_rgb_verified, "Hypothesis_1");
   PointInTPtr transformed_PCl (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::transformPointCloud (*grph[*vp.first].pScenePCl, *transformed_PCl, transformation);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified2 (transformed_PCl);
   vis_temp2->addPointCloud<pcl::PointXYZRGB> (transformed_PCl, handler_rgb_verified2, "Hypothesis_2");
   vis_temp2->spin ();*/
}

void
selectLowestWeightEdgesFromParallelEdges (const std::vector<Edge> &parallel_edges, const Graph &grph, std::vector<Edge> &single_edges)
{
  for (size_t edgeVec_id = 0; edgeVec_id < parallel_edges.size (); edgeVec_id++)
  {
    Vertex vrtx_src, vrtx_trgt;
    vrtx_src = source (parallel_edges[edgeVec_id], grph);
    vrtx_trgt = target (parallel_edges[edgeVec_id], grph);

    bool found = false;
    for (size_t edges_lowestWeight_id = 0; edges_lowestWeight_id < single_edges.size (); edges_lowestWeight_id++) //select edge with lowest weight amongst parallel edges
    {

      //check if edge already exists in final graph between these two vertices
      if ((((boost::get (vertex_index, grph, source (single_edges[edges_lowestWeight_id], grph)) == boost::get (vertex_index, grph, vrtx_src))
          && (boost::get (vertex_index, grph, target (single_edges[edges_lowestWeight_id], grph)) == boost::get (vertex_index, grph, vrtx_trgt)))
          || ((boost::get (vertex_index, grph, source (single_edges[edges_lowestWeight_id], grph)) == boost::get (vertex_index, grph, vrtx_trgt))
              && (boost::get (vertex_index, grph, target (single_edges[edges_lowestWeight_id], grph)) == boost::get (vertex_index, grph, vrtx_src)))))
      {
        found = true;
        if (grph[parallel_edges[edgeVec_id]].edge_weight < grph[single_edges[edges_lowestWeight_id]].edge_weight) //check for lowest edge cost - if lower than currently lowest weight, then replace
        {
          single_edges[edges_lowestWeight_id] = parallel_edges[edgeVec_id];
        }
        break;
      }
    }
    if (!found)
      single_edges.push_back (parallel_edges[edgeVec_id]);
  }
}

void
extendHypothesis (Graph &grph)
{
  bool something_has_been_updated = true;
  std::pair<vertex_iter, vertex_iter> vp; //vp.first = running iterator..... vp.second = last iterator
  while (something_has_been_updated)
  {
    something_has_been_updated = false;
    for (vp = vertices (grph); vp.first != vp.second; ++vp.first)
    {
      typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
      for (tie (out_i, out_end) = out_edges (*vp.first, grph); out_i != out_end; ++out_i)
      {
        Edge e = *out_i;
        Vertex src = source (e, grph), targ = target (e, grph);

        if (boost::get (vertex_index, grph, src) != boost::get (vertex_index, grph, *vp.first))
          std::cout << "something's wrong" << std::endl;

        size_t hypothesis_length_before_extension = grph[src].hypothesis.size ();

        for (std::vector<Hypothesis>::iterator it_hypB = grph[targ].hypothesis.begin (); it_hypB != grph[targ].hypothesis.end (); ++it_hypB)
        {
          bool hypotheses_from_view_exist = false;

          //---check-if-hypotheses-from-updating-view-already-exist-in-current-view------------------------
          for (size_t id_hypA = 0; id_hypA < hypothesis_length_before_extension; id_hypA++)
          {
            if (grph[src].hypothesis[id_hypA].origin_ == it_hypB->origin_)
            {
              hypotheses_from_view_exist = true;
            }
          }
          if (!hypotheses_from_view_exist)
          {
            Eigen::Matrix4f tf;
            if (grph[e].source_id == boost::get (vertex_index, grph, *vp.first))
            {
              tf = grph[e].transformation.inverse () * it_hypB->transform_;
            }
            else
            {
              tf = grph[e].transformation * it_hypB->transform_;
            }

            Hypothesis ht_temp (it_hypB->model_id_, tf, it_hypB->origin_, true);
            grph[*vp.first].hypothesis.push_back (ht_temp);
            something_has_been_updated = true;
          }
        }
      }
    }
  }
}

void
calcMST (const std::vector<Edge> &edges, const Graph &grph, std::vector<Edge> &edges_final)
{
  GraphMST grphMST;
  std::vector<VertexMST> verticesMST_v;
  std::vector<Edge> edges_lowestWeight;
  
  for (std::pair<vertex_iter, vertex_iter> vp = vertices (grph); vp.first != vp.second; ++vp.first)
  {
    VertexMST vrtxMST = boost::add_vertex (grphMST);
    verticesMST_v.push_back (vrtxMST);
  }

  selectLowestWeightEdgesFromParallelEdges (edges, grph, edges_lowestWeight);

  //---create-input-for-Minimum-Spanning-Tree-calculation-------------------------------
  for (size_t edgeVec_id = 0; edgeVec_id < edges_lowestWeight.size (); edgeVec_id++)
  {
    Vertex vrtx_src, vrtx_trgt;
    vrtx_src = source (edges_lowestWeight[edgeVec_id], grph);
    vrtx_trgt = target (edges_lowestWeight[edgeVec_id], grph);
    add_edge (verticesMST_v[get (vertex_index, grph, vrtx_src)], verticesMST_v[get (vertex_index, grph, vrtx_trgt)],
              grph[edges_lowestWeight[edgeVec_id]].edge_weight, grphMST);
  }

#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
  std::cout << "Boost Version not supported (you are using BOOST_MSVC Version: " << BOOST_MSVC << ", BOOST_MSVC > 1300 needed)" << std::endl;
#else
  std::vector < graph_traits<GraphMST>::vertex_descriptor > p (num_vertices (grphMST));
  prim_minimum_spanning_tree (grphMST, &p[0]);

  dynamic_properties dp;
  dp.property ("node_id", get (vertex_index, grphMST));
  dp.property ("weight", get (edge_weight, grphMST));
  std::cout << "Result Prims Algorithm: \n======================" << std::endl;
  write_graphviz_dp (std::cout, grphMST, dp, "node_id");
  std::cout << " There are " << boost::num_edges (grphMST) << " edges in the graph grph." << std::endl;

#endif

  for (std::size_t i = 0; i != p.size (); ++i)
  {
    if (p[i] != i)
      std::cout << "parent[" << i << "] = " << p[i] << std::endl;
    else
      std::cout << "parent[" << i << "] = no parent" << std::endl;
  }

  for (size_t edgeVec_id = 0; edgeVec_id < edges_lowestWeight.size (); edgeVec_id++)
  {
    Vertex vrtx_src, vrtx_trgt;
    vrtx_src = source (edges_lowestWeight[edgeVec_id], grph);
    vrtx_trgt = target (edges_lowestWeight[edgeVec_id], grph);
    if (p[boost::get (vertex_index, grph, vrtx_src)] == boost::get (vertex_index, grph, vrtx_trgt) || p[boost::get (vertex_index, grph, vrtx_trgt)]
        == boost::get (vertex_index, grph, vrtx_src)) //check if edge represents an edge of Prim's Minimum Spanning Tree
    {
      edges_final.push_back (edges_lowestWeight[edgeVec_id]);
    }
  }
}

void
createEdgesFromHypothesisMatch (const std::vector<Vertex> &vertices_v, Graph &grph, std::vector<Edge> &edges)
{
  for (std::vector<Vertex>::const_iterator it_vrtxA = vertices_v.begin (); it_vrtxA != vertices_v.end (); ++it_vrtxA)
  {
    for (size_t hypVec_id = 0; hypVec_id < grph[*it_vrtxA].hypothesis.size (); hypVec_id++)
    {
      for (std::vector<Vertex>::const_iterator it_vrtxB = vertices_v.begin (); it_vrtxB != it_vrtxA; ++it_vrtxB)
      {
        for (std::vector<Hypothesis>::iterator it_hypB = grph[*it_vrtxB].hypothesis.begin (); it_hypB != grph[*it_vrtxB].hypothesis.end (); ++it_hypB)
        {
          if (it_hypB->model_id_.compare (grph[*it_vrtxA].hypothesis[hypVec_id].model_id_) == 0) //model exists in other file -> create connection
          {
            Eigen::Matrix4f tf_temp = it_hypB->transform_ * grph[*it_vrtxA].hypothesis[hypVec_id].transform_.inverse (); //might be the other way around

            //link views by an edge (for other graph)
            Edge e_cpy;
            bool b;
            tie (e_cpy, b) = add_edge (*it_vrtxA, *it_vrtxB, grph);
            grph[e_cpy].transformation = tf_temp;
            grph[e_cpy].model_name = grph[*it_vrtxA].hypothesis[hypVec_id].model_id_;
            grph[e_cpy].source_id = boost::get (vertex_index, grph, *it_vrtxA);
            grph[e_cpy].target_id = boost::get (vertex_index, grph, *it_vrtxB);
            edges.push_back (e_cpy);

            std::cout << "Creating Edge from " << boost::get (vertex_index, grph, *it_vrtxA) << " to " << boost::get (vertex_index, grph, *it_vrtxB)
                << std::endl;
          }
        }
      }
    }
  }
}

View::View ()
{
  pScenePCl.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
  pScenePCl_f.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
  pSceneNormal.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pScenePCl_f_ds.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
  pIndices_above_plane.reset (new pcl::PointIndices);
  pSignatures.reset (new pcl::PointCloud<FeatureT>);
}

Hypothesis::Hypothesis (std::string model_id, Eigen::Matrix4f transform, std::string origin, bool extended, bool verified)
{
  model_id_ = model_id;
  transform_ = transform;
  origin_ = origin;
  extended_ = extended;
  verified_ = verified;
}

void
calcEdgeWeight (std::vector<Edge> &edges, Graph &grph, bool edge_weight_by_scene, bool edge_weight_by_projection)
{
  //----calculate-edge-weight---------------------------------------------------------
  //pcl::visualization::PCLVisualizer::Ptr vis_temp ( new pcl::visualization::PCLVisualizer );
  //std::vector<int> viewportNr_temp = visualization_framework ( vis_temp, 2, 1 );
#pragma omp parallel for
  for (size_t edge_id = 0; edge_id < edges.size (); edge_id++) //std::vector<Edge>::iterator edge_it = edges.begin(); edge_it!=edges.end(); ++edge_it)
  {

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pTargetNormalPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTargetPCl (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pSourcePCl (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSourceNormalPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr unusedCloud (new pcl::PointCloud<pcl::PointXYZ>);

    double edge_weight;
    Vertex vrtx_src, vrtx_trgt;
    vrtx_src = source (edges[edge_id], grph);
    vrtx_trgt = target (edges[edge_id], grph);

    Eigen::Matrix4f transform;
    if (grph[edges[edge_id]].source_id == boost::get (vertex_index, grph, vrtx_src))
    {
      transform = grph[edges[edge_id]].transformation;
    }
    else
    {
      transform = grph[edges[edge_id]].transformation.inverse ();
    }

    float w_after_icp_ = std::numeric_limits<float>::max ();

    if (!edge_weight_by_scene) //---merge-all-target-hypothesis-to-a-scene-cloud--------------------------------------- OTHER WAY AROUND SHOULD BE DONE TOO!
    {
      for (std::vector<Hypothesis>::iterator it_hypTarg = grph[vrtx_trgt].hypothesis.begin (); it_hypTarg != grph[vrtx_trgt].hypothesis.end (); ++it_hypTarg)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pModelPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pModelTargetPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::io::loadPCDFile (it_hypTarg->model_id_, *(pModelPCl));
        pcl::transformPointCloudWithNormals (*pModelPCl, *pModelTargetPCl, it_hypTarg->transform_);
        *pTargetNormalPCl += *pModelTargetPCl;
      }

      pcl::copyPointCloud (*pTargetNormalPCl, *pTargetPCl); // remove normals

      //---merge-all-source-hypothesis-to-a-scene-cloud---------------------------------------
      for (std::vector<Hypothesis>::iterator it_hypSrc = grph[vrtx_src].hypothesis.begin (); it_hypSrc != grph[vrtx_src].hypothesis.end (); ++it_hypSrc)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pModelPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pModelTargetPCl (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::io::loadPCDFile (it_hypSrc->model_id_, *(pModelPCl));
        pcl::transformPointCloudWithNormals (*pModelPCl, *pModelTargetPCl, transform * it_hypSrc->transform_);
        *pSourceNormalPCl += *pModelTargetPCl;
      }

      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr pOctree (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> (0.005f));
      pOctree->setInputCloud (pTargetPCl);
      pOctree->addPointsFromInputCloud ();
      edge_weight = calcRegistrationCost (pSourceNormalPCl, pTargetNormalPCl, pOctree, 1, 0.2);
    }
    else
    {

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTargetPCl_ficp (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pSourcePCl_ficp (new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::PassThrough<pcl::PointXYZRGB> pass_;
      pass_.setFilterLimits (0.f, 1.25f);
      pass_.setFilterFieldName ("z");
      pass_.setKeepOrganized (true);

      pass_.setInputCloud (grph[vrtx_src].pScenePCl);
      pass_.filter (*pSourcePCl_ficp);

      pass_.setInputCloud (grph[vrtx_trgt].pScenePCl);
      pass_.filter (*pTargetPCl_ficp);

      float best_overlap_ = 0.75f;
      Eigen::Matrix4f icp_trans;
      faat_pcl::registration::FastIterativeClosestPointWithGC<pcl::PointXYZRGB> icp;
      icp.setMaxCorrespondenceDistance (0.02f);
      icp.setInputSource (pSourcePCl_ficp);
      icp.setInputTarget (pTargetPCl_ficp);
      icp.setUseNormals (true);
      icp.useStandardCG (true);
      icp.setOverlapPercentage (best_overlap_);
      icp.setKeepMaxHypotheses (1);
      icp.setMaximumIterations (5);
      icp.align (transform);	// THERE IS A PROBLEM WITH THE VISUALIZER - CREATES VIS_Window WITHOUT ANY PURPOSE
      w_after_icp_ = icp.getFinalTransformation (icp_trans);
      if (w_after_icp_ < 0 || !pcl_isfinite(w_after_icp_))
      {
        w_after_icp_ = std::numeric_limits<float>::max ();
      }
      else
      {
        w_after_icp_ = best_overlap_ - w_after_icp_;
      }

      pcl::transformPointCloudWithNormals (*(grph[vrtx_src].pSceneNormal), *pTargetNormalPCl, icp_trans);
      pcl::copyPointCloud (*(grph[vrtx_trgt].pSceneNormal), *pSourceNormalPCl);

      /*pcl::transformPointCloud ( * ( grph[vrtx_src].pScenePCl_f ), *pTargetPCl, transform );
       pcl::copyPointCloud( * ( grph[vrtx_trgt].pScenePCl_f ), *pSourcePCl);
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTargetPCl_ficp ( new pcl::PointCloud<pcl::PointXYZRGB> );
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pSourcePCl_ficp ( new pcl::PointCloud<pcl::PointXYZRGB> );
       {
       std::vector<int> indices_p;
       pcl::removeNaNFromPointCloud(*pTargetPCl, *pTargetPCl_ficp, indices_p);
       }

       {
       std::vector<int> indices_p;
       pcl::removeNaNFromPointCloud(*pSourcePCl, *pSourcePCl_ficp, indices_p);
       }

       float leaf = 0.005f;
       pcl::VoxelGrid < pcl::PointXYZRGB > sor;
       sor.setLeafSize ( leaf, leaf, leaf );

       sor.setInputCloud ( pSourcePCl_ficp );
       sor.filter ( *pSourcePCl );

       sor.setInputCloud ( pTargetPCl_ficp );
       sor.filter ( *pTargetPCl );

       pcl::IterativeClosestPoint < pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
       icp.setInputSource (pTargetPCl);
       icp.setInputTarget (pSourcePCl);
       icp.setMaxCorrespondenceDistance(0.02f);
       icp.setMaximumIterations(20);
       icp.setRANSACIterations(100000);
       icp.setRANSACOutlierRejectionThreshold(0.01f);
       icp.setEuclideanFitnessEpsilon(1e-12);
       icp.setTransformationEpsilon(1e-12);
       pcl::PointCloud < pcl::PointXYZRGB >::Ptr Final( new pcl::PointCloud<pcl::PointXYZRGB> );
       icp.align (*Final);

       Eigen::Matrix4f icp_trans;
       icp_trans = icp.getFinalTransformation();

       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pTargetNormalPCl_tmp ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
       pcl::transformPointCloudWithNormals ( *pTargetNormalPCl, *pTargetNormalPCl_tmp, icp_trans );
       pcl::copyPointCloud ( * pTargetNormalPCl_tmp, *pTargetNormalPCl );*/

      if (grph[edges[edge_id]].source_id == boost::get (vertex_index, grph, vrtx_src))
      {
        PCL_WARN("Normal...\n");
        //icp trans is aligning source to target
        //transform is aligning source to target
        //grph[edges[edge_id]].transformation = icp_trans * grph[edges[edge_id]].transformation;
        grph[edges[edge_id]].transformation = icp_trans;
      }
      else
      {
        //transform is aligning target to source
        //icp trans is aligning source to target
        PCL_WARN("Inverse...\n");
        //grph[edges[edge_id]].transformation = icp_trans.inverse() * grph[edges[edge_id]].transformation;
        grph[edges[edge_id]].transformation = icp_trans.inverse ();
      }
    }

    if (!edge_weight_by_projection)
    {
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr pOctree (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> (0.005f));
      pOctree->setInputCloud (pTargetPCl);
      pOctree->addPointsFromInputCloud ();
      edge_weight = calcRegistrationCost (pSourceNormalPCl, pTargetNormalPCl, pOctree, 1, 0.2);
    }
    else
    {
      //pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr pOrganizedNeighbor (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>);
      // 	    pOrganizedNeighbor->setInputCloud ( pTargetPCl );
      // 	    std::cout << "Is PointCloud organized? " << static_cast<int> (pTargetPCl->isOrganized()) << std::endl;
      // 	    std::cout << "Is Search-Object valid? " << static_cast<int> (pOrganizedNeighbor->isValid()) << std::endl;
      // 	    edge_weight = calcRegistrationCost ( pSourceNormalPCl, pTargetNormalPCl,  pOrganizedNeighbor, 1, 0.2 );
      //edge_weight = calcRegistrationCost ( pSourceNormalPCl, pTargetNormalPCl, 0.2 );

      /*std::vector<int> unused;
       edge_weight = calcRegistrationCost ( pTargetNormalPCl, pSourceNormalPCl, unused, 0.2 );
       pcl::copyPointCloud(*pTargetNormalPCl, unused, *unusedCloud);*/

      edge_weight = w_after_icp_;
    }

    std::cout << "WEIGHT IS: " << edge_weight << " coming from edge with object_id: " << grph[edges[edge_id]].model_name << std::endl;
    grph[edges[edge_id]].edge_weight = edge_weight;

    /*vis_temp->removeAllPointClouds();
     pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGBNormal > handler_rgb_verified (pSourceNormalPCl);
     vis_temp->addPointCloud<pcl::PointXYZRGBNormal> (pSourceNormalPCl, handler_rgb_verified, "Hypothesis_1");
     pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGBNormal > handler_rgb_verified2 (pTargetNormalPCl);
     vis_temp->addPointCloud<pcl::PointXYZRGBNormal> (pTargetNormalPCl, handler_rgb_verified2, "Hypothesis_2");

     if(unusedCloud->points.size() > 0)
     {
     pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_rgb_verified2 (unusedCloud, 255, 0, 0);
     vis_temp->addPointCloud<pcl::PointXYZ> (unusedCloud, handler_rgb_verified2, "unused");
     }
     vis_temp->spin();*/
  }
}

double
calcRegistrationCost (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,
                      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr pOctree, int K, double beta)
{
  //calculates the discrepancy between the inputPCl and the ScenePCL by K nearest neighbor search.
  //.........................

  double overlap = 0;
  double pt2ptDistSUM = 0;
  double dot_prodSUM = 0;
  double color_regSUM = 0;
  int n_points_used = 0;

  std::vector<int> scene_points_indices;
  for (size_t ptId = 0; ptId < pInputNormalPCl->points.size (); ++ptId)
  {
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    pcl::PointXYZRGB searchPoint;
    if (!(isnan (pInputNormalPCl->points[ptId].normal_x) || isnan (pInputNormalPCl->points[ptId].normal_y)
        || isnan (pInputNormalPCl->points[ptId].normal_z)))
    {
      searchPoint.getVector4fMap () = pInputNormalPCl->points[ptId].getVector4fMap ();

      if (pOctree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {

        for (size_t idKnn = 0; idKnn < pointIdxNKNSearch.size (); ++idKnn)
        {

          if (!(isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_x)
              || isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_y)
              || isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_z)))
          {
            float rgb_m, rgb_s;
            rgb_m = pInputNormalPCl->points[ptId].rgb;
            rgb_s = pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].rgb;
            uint32_t rgb = *reinterpret_cast<int*> (&rgb_m);
            uint8_t rm = (rgb >> 16) & 0x0000ff;
            uint8_t gm = (rgb >> 8) & 0x0000ff;
            uint8_t bm = (rgb) & 0x0000ff;

            rgb = *reinterpret_cast<int*> (&rgb_s);
            uint8_t rs = (rgb >> 16) & 0x0000ff;
            uint8_t gs = (rgb >> 8) & 0x0000ff;
            uint8_t bs = (rgb) & 0x0000ff;

            float ym = 0.257f * rm + 0.504f * gm + 0.098f * bm + 16; //between 16 and 235
            float um = -(0.148f * rm) - (0.291f * gm) + (0.439f * bm) + 128;
            float vm = (0.439f * rm) - (0.368f * gm) - (0.071f * bm) + 128;

            float ys = 0.257f * rs + 0.504f * gs + 0.098f * bs + 16; //between 16 and 235
            float us = -(0.148f * rs) - (0.291f * gs) + (0.439f * bs) + 128;
            float vs = (0.439f * rs) - (0.368f * gs) - (0.071f * bs) + 128;
            float color_sigma = 35.f;
            color_sigma *= color_sigma;
            float color_reg = std::exp ((-0.5f * (um - us) * (um - us)) / (color_sigma)) * std::exp ((-0.5f * (vm - vs) * (vm - vs)) / (color_sigma));

            float pt2ptDist = sqrt (pointNKNSquaredDistance[idKnn]);
            float dot_prod =
                pInputNormalPCl->points[ptId].getNormalVector3fMap ().dot (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].getNormalVector3fMap ()); // vectors_dot_prod(pModelPCl_aligned_icp_wNormal->points[ptId].normal, .normal, 3);
            //scene_points_indices.push_back(pointIdxNKNSearch[idKnn]);

            double thresholdPt2PtDist = 0.01;
            if (pt2ptDist > thresholdPt2PtDist)
            {
              pt2ptDist = thresholdPt2PtDist;
              continue;
              //insert Threshold
            }

            overlap += /*pt2ptDist/thresholdPt2PtDist +*/beta * (1.f - dot_prod) + (1.f - color_reg);
            pt2ptDistSUM += pt2ptDist / thresholdPt2PtDist;
            dot_prodSUM += beta * (1.f - dot_prod);
            color_regSUM += (1.f - color_reg);
            n_points_used++;
          }
        }
      }
    }
  }

  //std::sort(scene_points_indices.begin(),scene_points_indices.end());
  //scene_points_indices.erase( std::unique( scene_points_indices.begin(), scene_points_indices.end() ), scene_points_indices.end() );
  //pcl::copyPointCloud(*grph[*it_vrtx].pScenePCl, scene_points_indices, pScenePCl_knn);

  std::cout << "N points used: " << n_points_used << ";  ptp2ptDist: " << pt2ptDistSUM / n_points_used << ";   dot_prod: " << dot_prodSUM
      / n_points_used << ";   color: " << color_regSUM / n_points_used << std::endl;
  return overlap / static_cast<float> (n_points_used);
}

double
calcRegistrationCost (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,
                      pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr pOrganizedNeighbor, int K, double beta)
{
  //calculates the discrepancy between the inputPCl and the ScenePCL by K nearest neighbor search.
  //.........................

  double overlap = 0;
  double pt2ptDistSUM = 0;
  double dot_prodSUM = 0;
  double color_regSUM = 0;
  int n_points_used = 0;

  std::vector<int> scene_points_indices;
  //#pragma omp parallel for
  for (size_t ptId = 0; ptId < pInputNormalPCl->points.size (); ++ptId)
  {

    if (!(isnan (pInputNormalPCl->points[ptId].x) || isnan (pInputNormalPCl->points[ptId].y) || isnan (pInputNormalPCl->points[ptId].z)
        || isnan (pInputNormalPCl->points[ptId].normal_x) || isnan (pInputNormalPCl->points[ptId].normal_y)
        || isnan (pInputNormalPCl->points[ptId].normal_z)))
    {
      std::vector<int> pointIdxNKNSearch;
      std::vector<float> pointNKNSquaredDistance;

      pcl::PointXYZRGB searchPoint;
      searchPoint.getVector4fMap () = pInputNormalPCl->points[ptId].getVector4fMap ();

      if (pOrganizedNeighbor->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {

        for (size_t idKnn = 0; idKnn < pointIdxNKNSearch.size (); ++idKnn)
        {

          if (!(isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_x)
              || isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_y)
              || isnan (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].normal_z)))
          {
            float rgb_m, rgb_s;
            rgb_m = pInputNormalPCl->points[ptId].rgb;
            rgb_s = pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].rgb;
            uint32_t rgb = *reinterpret_cast<int*> (&rgb_m);
            uint8_t rm = (rgb >> 16) & 0x0000ff;
            uint8_t gm = (rgb >> 8) & 0x0000ff;
            uint8_t bm = (rgb) & 0x0000ff;

            rgb = *reinterpret_cast<int*> (&rgb_s);
            uint8_t rs = (rgb >> 16) & 0x0000ff;
            uint8_t gs = (rgb >> 8) & 0x0000ff;
            uint8_t bs = (rgb) & 0x0000ff;

            float ym = 0.257f * rm + 0.504f * gm + 0.098f * bm + 16; //between 16 and 235
            float um = -(0.148f * rm) - (0.291f * gm) + (0.439f * bm) + 128;
            float vm = (0.439f * rm) - (0.368f * gm) - (0.071f * bm) + 128;

            float ys = 0.257f * rs + 0.504f * gs + 0.098f * bs + 16; //between 16 and 235
            float us = -(0.148f * rs) - (0.291f * gs) + (0.439f * bs) + 128;
            float vs = (0.439f * rs) - (0.368f * gs) - (0.071f * bs) + 128;
            float color_sigma = 35.f;
            color_sigma *= color_sigma;
            float color_reg = std::exp ((-0.5f * (um - us) * (um - us)) / (color_sigma)) * std::exp ((-0.5f * (vm - vs) * (vm - vs)) / (color_sigma));

            float pt2ptDist = sqrt (pointNKNSquaredDistance[idKnn]);
            float dot_prod =
                pInputNormalPCl->points[ptId].getNormalVector3fMap ().dot (pSceneNormalPCl->points[pointIdxNKNSearch[idKnn]].getNormalVector3fMap ()); // vectors_dot_prod(pModelPCl_aligned_icp_wNormal->points[ptId].normal, .normal, 3);
            //scene_points_indices.push_back(pointIdxNKNSearch[idKnn]);

            double thresholdPt2PtDist = 0.01;
            if (pt2ptDist > thresholdPt2PtDist)
            { //insert Threshold
              continue;
              pt2ptDist = thresholdPt2PtDist;
            }

            overlap += pt2ptDist / thresholdPt2PtDist + beta * (1.f - dot_prod) + (1.f - color_reg);
            pt2ptDistSUM += pt2ptDist / thresholdPt2PtDist;
            dot_prodSUM += beta * (1.f - dot_prod);
            color_regSUM += (1.f - color_reg);
            n_points_used++;
          }
        }
      }
    }
  }

  //std::sort(scene_points_indices.begin(),scene_points_indices.end());
  //scene_points_indices.erase( std::unique( scene_points_indices.begin(), scene_points_indices.end() ), scene_points_indices.end() );
  //pcl::copyPointCloud(*grph[*it_vrtx].pScenePCl, scene_points_indices, pScenePCl_knn);

  std::cout << "N points used: " << n_points_used << ";  ptp2ptDist: " << pt2ptDistSUM / n_points_used << ";   dot_prod: " << dot_prodSUM
      / n_points_used << ";   color: " << color_regSUM / n_points_used << std::endl;
  return overlap / static_cast<float> (n_points_used);
}

double
calcRegistrationCost (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInputNormalPCl, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pSceneNormalPCl,
                      std::vector<int> & unused, double beta)
{
  //calculates the discrepancy between the inputPCl and the ScenePCL by K nearest neighbor search.
  //.........................

  double overlap = 0;
  double pt2ptDistSUM = 0;
  double dot_prodSUM = 0;
  double color_regSUM = 0;
  int n_points_used = 0;

  std::vector<int> scene_points_indices;
  //#pragma omp parallel for
  for (size_t ptId = 0; ptId < pInputNormalPCl->points.size (); ++ptId)
  {

    if (!(isnan (pInputNormalPCl->points[ptId].x) || isnan (pInputNormalPCl->points[ptId].y) || isnan (pInputNormalPCl->points[ptId].z)
        || isnan (pInputNormalPCl->points[ptId].normal_x) || isnan (pInputNormalPCl->points[ptId].normal_y)
        || isnan (pInputNormalPCl->points[ptId].normal_z)))
    {
      int u, v;
      float fl = 525.f;
      float cx, cy;
      cx = 320.f;
      cy = 240.f;
      u = fl * (pInputNormalPCl->points[ptId].x / pInputNormalPCl->points[ptId].z) + cx;
      v = fl * (pInputNormalPCl->points[ptId].y / pInputNormalPCl->points[ptId].z) + cy;

      if (u >= pSceneNormalPCl->width || v >= pSceneNormalPCl->height || u < 0 || v < 0)
      {
        unused.push_back (static_cast<int> (ptId));
        continue;
      }

      pcl::PointXYZRGBNormal pt = pSceneNormalPCl->at (u, v);

      if (!(isnan (pt.x) || isnan (pt.y) || isnan (pt.z) || isnan (pt.normal_x) || isnan (pt.normal_y) || isnan (pt.normal_z)))
      {
        float pt2ptDist = (pt.getVector3fMap () - pInputNormalPCl->points[ptId].getVector3fMap ()).norm ();
        float dot_prod = pInputNormalPCl->points[ptId].getNormalVector3fMap ().dot (pt.getNormalVector3fMap ());
        float rgb_m, rgb_s;
        rgb_m = pInputNormalPCl->points[ptId].rgb;
        rgb_s = pt.rgb;
        uint32_t rgb = *reinterpret_cast<int*> (&rgb_m);
        uint8_t rm = (rgb >> 16) & 0x0000ff;
        uint8_t gm = (rgb >> 8) & 0x0000ff;
        uint8_t bm = (rgb) & 0x0000ff;

        rgb = *reinterpret_cast<int*> (&rgb_s);
        uint8_t rs = (rgb >> 16) & 0x0000ff;
        uint8_t gs = (rgb >> 8) & 0x0000ff;
        uint8_t bs = (rgb) & 0x0000ff;

        float ym = 0.257f * rm + 0.504f * gm + 0.098f * bm + 16; //between 16 and 235
        float um = -(0.148f * rm) - (0.291f * gm) + (0.439f * bm) + 128;
        float vm = (0.439f * rm) - (0.368f * gm) - (0.071f * bm) + 128;

        float ys = 0.257f * rs + 0.504f * gs + 0.098f * bs + 16; //between 16 and 235
        float us = -(0.148f * rs) - (0.291f * gs) + (0.439f * bs) + 128;
        float vs = (0.439f * rs) - (0.368f * gs) - (0.071f * bs) + 128;
        float color_sigma = 35.f;
        color_sigma *= color_sigma;
        float color_reg = std::exp ((-0.5f * (um - us) * (um - us)) / (color_sigma)) * std::exp ((-0.5f * (vm - vs) * (vm - vs)) / (color_sigma));

        double thresholdPt2PtDist = 0.01;
        if (pt2ptDist > thresholdPt2PtDist)
        {//insert Threshold
          //std::cout << pt2ptDist << std::endl;
          unused.push_back (static_cast<int> (ptId));
          continue;
          pt2ptDist = thresholdPt2PtDist;
        }

        overlap += pt2ptDist / thresholdPt2PtDist + beta * (1.f - dot_prod) + (1.f - color_reg);
        pt2ptDistSUM += pt2ptDist / thresholdPt2PtDist;
        dot_prodSUM += beta * (1.f - dot_prod);
        color_regSUM += (1.f - color_reg);
        n_points_used++;
      }
      else
      {
        unused.push_back (static_cast<int> (ptId));
      }
    }
  }

  //std::sort(scene_points_indices.begin(),scene_points_indices.end());
  //scene_points_indices.erase( std::unique( scene_points_indices.begin(), scene_points_indices.end() ), scene_points_indices.end() );
  //pcl::copyPointCloud(*grph[*it_vrtx].pScenePCl, scene_points_indices, pScenePCl_knn);

  std::cout << "N points used: " << n_points_used << ";  ptp2ptDist: " << pt2ptDistSUM / n_points_used << ";   dot_prod: " << dot_prodSUM
      / n_points_used << ";   color: " << color_regSUM / n_points_used << std::endl;
  return overlap / static_cast<float> (n_points_used) + (1.f - std::min (0.5f, n_points_used / static_cast<float> (unused.size ())));
}



void multiviewGraph::chatterCallback(const visual_recognizer::Hypotheses& msg)
{
  ROS_INFO("I got %lu hypotheses.", msg.data.size());
  if( msg.data.size()>0 )
  {
    Vertex vrtx = boost::add_vertex (grph);
    vertices_v.push_back (vrtx);
    grph[vrtx].scene_filename = "PLEASE CHANGE THIS";
    
    for(size_t hypId = 0; hypId < msg.data.size(); hypId++)
    {
      Eigen::Matrix4f transform;
      transform(0,0) = msg.data[hypId].transform[0];
      transform(0,1) = msg.data[hypId].transform[1];
      transform(0,2) = msg.data[hypId].transform[2];
      transform(1,0) = msg.data[hypId].transform[3];
      transform(1,1) = msg.data[hypId].transform[4];
      transform(1,2) = msg.data[hypId].transform[5];
      transform(2,0) = msg.data[hypId].transform[6];
      transform(2,1) = msg.data[hypId].transform[7];
      transform(2,2) = msg.data[hypId].transform[8];
      transform(0,3) = msg.data[hypId].transform[6];
      transform(1,3) = msg.data[hypId].transform[7];
      transform(2,3) = msg.data[hypId].transform[8];
      transform(3,0) = 0;
      transform(3,1) = 0;
      transform(3,2) = 0;
      transform(3,3) = 1;
     
      Hypothesis hypothesis (msg.data[hypId].model_id, transform, grph[vrtx].scene_filename, false);
      grph[vrtx].hypothesis.push_back (hypothesis);
    }
  }
    for (std::vector<Vertex>::iterator it_vrtxA = vertices_v.begin (); it_vrtxA != vertices_v.end (); ++it_vrtxA)
  {
    Vertex vrtx_final = boost::add_vertex (grph_final);
    grph[*it_vrtxA].pIndices_above_plane.reset(new pcl::PointIndices);		//Don't know why this does not work with the constructor?!
    //----create-filter-that-passes-only-points-within-a-certain-distance-and-above-table-plane----
    filterPCl (grph[*it_vrtxA].pScenePCl, grph[*it_vrtxA].pScenePCl_f, grph[*it_vrtxA].pIndices_above_plane, 1.15);

    //--- estimate surface normals of the scene --------
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (grph[*it_vrtxA].pScenePCl_f);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01); // Use all neighbors in a sphere of radius 1cm
    ne.compute (*grph[*it_vrtxA].pSceneNormal); // Compute the features

#pragma omp parallel for
    for (size_t i = 0; i < grph[*it_vrtxA].pSceneNormal->points.size (); i++)
    {
      grph[*it_vrtxA].pSceneNormal->points[i].x = grph[*it_vrtxA].pScenePCl_f->points[i].x;
      grph[*it_vrtxA].pSceneNormal->points[i].y = grph[*it_vrtxA].pScenePCl_f->points[i].y;
      grph[*it_vrtxA].pSceneNormal->points[i].z = grph[*it_vrtxA].pScenePCl_f->points[i].z;
      grph[*it_vrtxA].pSceneNormal->points[i].rgb = grph[*it_vrtxA].pScenePCl_f->points[i].rgb;
    }

    //---copy-vertex-and-its-information-to-graph_final----------------------------
    grph_final[vrtx_final].pScenePCl = grph[*it_vrtxA].pScenePCl;
    grph_final[vrtx_final].pScenePCl_f = grph[*it_vrtxA].pScenePCl_f;
    grph_final[vrtx_final].pSceneNormal = grph[*it_vrtxA].pSceneNormal;
    //grph_final[vrtx_final].pScenePCl_f_ds = grph[*it_vrtx].pScenePCl_f_ds;
    grph_final[vrtx_final].scene_filename = grph[*it_vrtxA].scene_filename;
    grph_final[vrtx_final].hypothesis = grph[*it_vrtxA].hypothesis;
  }
  
    createEdgesFromHypothesisMatch (vertices_v, grph, edges);
}
