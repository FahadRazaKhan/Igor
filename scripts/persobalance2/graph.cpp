#include <Eigen/Core>
#include <Eigen/LU>

// #include <iostream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/property_map/vector_property_map.hpp>

#include "graph.hpp"


struct graph {

  struct vertex_property;
  struct edge_property;
  
  typedef boost::adjacency_list<boost::vecS,
								boost::vecS,
								boost::undirectedS,
								vertex_property,
								edge_property> bgl_type;

  struct vertex_property {
	unsigned dim;
	
	unsigned vector_offset, matrix_offset, matrix_inv_offset;
	mutable vertex parent;
  };

  struct edge_property {
	unsigned matrix_offset;
  };
  
  
  bgl_type bgl;
  
  std::vector<graph_data_type> storage;
  
  std::vector<unsigned> top_down; // top-down depth-first traversal order
  std::vector<boost::default_color_type> color_map;

  // row-major matrices to ease numpy interaction
  typedef Eigen::Matrix<graph_data_type, Eigen::Dynamic, Eigen::Dynamic,
						Eigen::RowMajor> matrix_type;
  
  typedef Eigen::Matrix<graph_data_type, Eigen::Dynamic, 1> vector_type;
  
  typedef Eigen::Map< matrix_type > matrix_view_type;
  typedef Eigen::Map< vector_type > vector_view_type;  
  
  matrix_view_type vertex_matrix(vertex v, unsigned* size = 0) {
	const unsigned s = bgl[v].dim;
	if(size) *size = s;
	return matrix_view_type(&storage[bgl[v].matrix_offset], s, s);
  }

  matrix_view_type vertex_matrix_inv(vertex v, unsigned* size = 0) {
	const unsigned s = bgl[v].dim;
	if(size) *size = s;
	return matrix_view_type(&storage[bgl[v].matrix_inv_offset], s, s);
  }

  
  matrix_view_type edge_matrix(bgl_type::edge_descriptor e, unsigned* rows = 0, unsigned* cols = 0) {
	const vertex s = source(e, bgl);
	const vertex t = target(e, bgl);
	
	assert(s > t && "edge is not lower-diagonal");

	const unsigned r = bgl[s].dim;
	const unsigned c = bgl[t].dim;

	if(rows) *rows = r;
	if(cols) *cols = c;

	return matrix_view_type(&storage[ bgl[e].matrix_offset ], r, c);
  }

  // convenience
  matrix_view_type edge_matrix(vertex s, vertex t, unsigned* rows = 0, unsigned* cols = 0) {
	auto e = boost::edge(s, t, bgl); assert(e.second && "invalid edge");
	return edge_matrix(e.first, rows, cols);
  }
  
  vector_view_type vertex_vector(vertex v, unsigned* size = 0) {
	const unsigned s = bgl[v].dim;
	if(size) *size = s;

	return vector_view_type(&storage[ bgl[v].vector_offset ], s);
  }
  
};


graph* graph_new() {
  return new graph;
}


void graph_delete(graph* g) {
  delete g;
}


void graph_clear(graph* g) {
  g->bgl.clear();
}

unsigned graph_num_vertices(graph* g) {
  return num_vertices(g->bgl);
}


vertex graph_add_vertex(graph* g, unsigned dim) {
  const vertex v = add_vertex(g->bgl);
  g->bgl[v].dim = dim;
  return v;
}


unsigned graph_vertex_dim(graph* g, vertex v) {
  return g->bgl[v].dim;
}


void graph_add_edge(graph* g, vertex from, vertex to) {
  std::pair<graph::bgl_type::edge_descriptor, bool> res = add_edge(from, to, g->bgl);

  assert(res.second);
  assert(from > to && "edge is not lower-diagonal");
  
}


void graph_init(graph* g) {

  g->storage.clear();
  
  // vertices
  for(unsigned i = 0, n = num_vertices(g->bgl); i < n; ++i) {
	graph::vertex_property& p = g->bgl[i];
	
	p.vector_offset = g->storage.size();
	g->storage.resize(p.vector_offset + p.dim);
	
	p.matrix_offset = g->storage.size();
	g->storage.resize(p.matrix_offset + p.dim * p.dim);

	p.matrix_inv_offset = g->storage.size();
	g->storage.resize(p.matrix_inv_offset + p.dim * p.dim);
  }

  // edges
  for(auto it = edges(g->bgl); it.first != it.second; ++it.first) {
	const graph::bgl_type::edge_descriptor e = *it.first;
	graph::edge_property& p = g->bgl[e];
	
	const unsigned rows = g->bgl[ source(e, g->bgl) ].dim;
	const unsigned cols = g->bgl[ target(e, g->bgl) ].dim;	
	
	p.matrix_offset = g->storage.size();
	g->storage.resize(p.matrix_offset + rows * cols);
  }

}




void graph_vertex_matrix(graph* g, vertex v, graph_data_type** data) {
  *data = &g->storage[g->bgl[v].matrix_offset];
}


void graph_edge_matrix(graph* g, vertex s, vertex t, graph_data_type** data) {
  auto e = boost::edge(s, t, g->bgl);
  assert(e.second && "invalid edge");
  assert( s > t && "edge must be lower-diagonal");
  
  *data = &g->storage[g->bgl[e.first].matrix_offset];
}


void graph_vertex_vector(graph* g, vertex v, graph_data_type** data) {
  *data = &g->storage[g->bgl[v].vector_offset];
}




template<class G, class Iterator, class F>
static void traverse(const G& g, Iterator first, Iterator last, const F& f) {

  for(Iterator it = first; it != last; ++it) {
	const vertex v = *it;
	const vertex p = g[v].parent;

	// std::cout << "traversing vertex: " << v << " parent: " << p << std::endl;
	
	// foreach child
	for(auto a = adjacent_vertices(v, g); a.first != a.second; ++a.first) {
	  
	  // parent
	  if( *a.first == p) continue;
	  
	  // child
	  const vertex c = *a.first;

	  f.out_edge(v, c);
	}

	f.finish(v);
	
	if(p != v) {
	  f.in_edge(p, v);
	}
  }

}


struct factor {

  graph* g;

  factor(graph* g) : g(g) { }
  
  void out_edge(vertex v, vertex c) const {
	// std::cout << "out_edge: " << v << " -> " << c << std::endl;
	
	const bool lower = v > c;
	const bool flip = !lower;

	graph::matrix_view_type H_v = g->vertex_matrix(v);
	const graph::matrix_view_type H_c = g->vertex_matrix(c);

	if( flip ) {
	  const graph::matrix_view_type J_cv = g->edge_matrix( boost::edge(c, v, g->bgl).first );
	  assert( boost::edge(c, v, g->bgl).second );

	  H_v.noalias() -= J_cv.transpose() * H_c * J_cv;
	} else {
	  const graph::matrix_view_type J_vc = g->edge_matrix( boost::edge(v, c, g->bgl).first);
	  assert( boost::edge(v, c, g->bgl).second );
	  
	  H_v.noalias() -= J_vc * H_c * J_vc.transpose();
	}

  }


  void finish(vertex v) const {
	// std::cout << "finish: " << v << std::endl;
	const graph::matrix_view_type H_v = g->vertex_matrix(v);
	graph::matrix_view_type Hinv_v = g->vertex_matrix_inv(v);

	// TODO use LDLT here
	Hinv_v.noalias() = H_v.inverse();
  }


  void in_edge(vertex p, vertex v) const {
	// std::cout << "in_edge: " << p << " -> " << v << std::endl;
	
	const bool lower = p > v;
	const bool flip = !lower;

	const graph::matrix_view_type Hinv_v = g->vertex_matrix_inv(v);

	if( flip ) {
	  graph::matrix_view_type J_vp = g->edge_matrix( boost::edge(v, p, g->bgl).first);
	  assert( boost::edge(v, p, g->bgl).second );

	  // TODO temporary
	  J_vp = Hinv_v * J_vp;
	} else {
	  graph::matrix_view_type J_pv = g->edge_matrix( boost::edge(p, v, g->bgl).first);
	  assert( boost::edge(p, v, g->bgl).second );

	  // TODO temporary
	  J_pv = J_pv * Hinv_v;
	}


  }

};


struct visitor : boost::default_dfs_visitor
{
  std::vector<unsigned>& top_down;
  visitor(std::vector<unsigned>& top_down)
	: top_down(top_down) {	}
	
public:

  void initialize_vertex(unsigned v, const graph::bgl_type& g) const {
	g[v].parent = v;
  }
  
  void discover_vertex(unsigned v, const graph::bgl_type& ) const {
	top_down.push_back(v);
  }


  void tree_edge(graph::bgl_type::edge_descriptor e, const graph::bgl_type& g) const {
	g[ target(e, g) ].parent = source(e, g);
  }
	
};


struct solve_first {
  graph* g;
  
  solve_first(graph* g) : g(g) { }


  void finish(vertex ) const { }
  void in_edge(vertex , vertex ) const { }  
  
  void out_edge(vertex v, vertex c) const {
	const bool lower = v > c;
	const bool flip = !lower;

	graph::vector_view_type x_v = g->vertex_vector(v);
	const graph::vector_view_type x_c = g->vertex_vector(c);

	if( flip ) {
	  const graph::matrix_view_type J_cv = g->edge_matrix( boost::edge(c, v, g->bgl).first);
	  assert( boost::edge(c, v, g->bgl).second );

	  x_v.noalias() -= J_cv.transpose() * x_c;
	} else {
	  const graph::matrix_view_type J_vc = g->edge_matrix( boost::edge(v, c, g->bgl).first);
	  assert( boost::edge(v, c, g->bgl).second );

	  x_v.noalias() -= J_vc * x_c;
	}

  }
  
};


struct solve_second {
  graph* g;
  
  solve_second(graph* g) : g(g) { }

  void out_edge(vertex , vertex ) const { }
  
  void finish(vertex v) const {

	graph::vector_view_type x_v = g->vertex_vector(v);
	const graph::matrix_view_type Hinv_v = g->vertex_matrix_inv(v);

	// TODO temporary
	x_v = Hinv_v * x_v;
  }

  void in_edge(vertex p, vertex v) const {
	graph::vector_view_type x_v = g->vertex_vector(v);
	const graph::vector_view_type x_p = g->vertex_vector(p);
	
	const bool lower = p > v;
	const bool flip = !lower;

	if( flip ) {
	  graph::matrix_view_type J_vp = g->edge_matrix( boost::edge(v, p, g->bgl).first);
	  assert( boost::edge(v, p, g->bgl).second );
	  
	  x_v.noalias() -= J_vp * x_p;
	} else {
	  graph::matrix_view_type J_pv = g->edge_matrix( boost::edge(p, v, g->bgl).first);
	  assert( boost::edge(p, v, g->bgl).second );

	  x_v.noalias() -= J_pv.transpose() * x_p;
	}
  }
  
};




void graph_factor(graph* g, unsigned root) {
  const unsigned n = num_vertices(g->bgl);
  
  g->top_down.clear();
  g->color_map.resize(n);
  
  depth_first_search(g->bgl, boost::visitor( visitor(g->top_down))
					 .color_map(boost::make_iterator_property_map(g->color_map.begin(),
																  get(boost::vertex_index, g->bgl)))
					 .root_vertex(root));
  assert(g->top_down.size() == n);

  // bottom-up
  traverse(g->bgl, g->top_down.rbegin(), g->top_down.rend(), factor(g) );
}


void graph_solve(graph* g) {
  
  // bottom-up
  traverse(g->bgl, g->top_down.rbegin(), g->top_down.rend(), solve_first(g) );

  // top-down
  traverse(g->bgl, g->top_down.begin(), g->top_down.end(), solve_second(g) );  
}

