#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "export.hpp"

extern "C" {

  struct graph;
  
  typedef unsigned long vertex;
  typedef double graph_data_type;

  EXPORT graph* graph_new();
  EXPORT void graph_delete(graph*);
  EXPORT void graph_clear(graph* g);

  EXPORT unsigned graph_num_vertices(graph* g);
  EXPORT unsigned graph_vertex_dim(graph* g, vertex v);
  
  EXPORT vertex graph_add_vertex(graph* g, unsigned dim);
  EXPORT void graph_add_edge(graph* g, vertex from, vertex to);

  // call this before accessing matrix/vector buffers
  EXPORT void graph_init(graph*);
  
  // data points to a symmetric dim(v) * dim(v) matrix
  EXPORT void graph_vertex_matrix(graph* g, vertex v, graph_data_type** data);
  
  // data points to a dim(s) * dim(t) row-major matrix. need s > t.
  EXPORT void graph_edge_matrix(graph* g, vertex s, vertex t, graph_data_type** data);

  // data points to a dim(v) vector
  EXPORT void graph_vertex_vector(graph* g, vertex v, graph_data_type** data);
  
  EXPORT void graph_factor(graph* g, unsigned root);
  EXPORT void graph_solve(graph* g);
  
}


#endif

