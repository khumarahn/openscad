// This file is a part of openscad. Everything implied is implied.
// Author: Alexey Korepanov <kaikaikai@yandex.ru>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>

#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <map>

#include "GeometryUtils.h"
#include "clipper-utils.h"
#include "roof_ss.h"

namespace roof_ss {

typedef CGAL::Exact_predicates_inexact_constructions_kernel         CGAL_KERNEL;
typedef CGAL_KERNEL::Point_2                                        CGAL_Point_2;
typedef CGAL::Polygon_2<CGAL_KERNEL>                                CGAL_Polygon_2;
typedef CGAL::Vector_2<CGAL_KERNEL>                                 CGAL_Vector_2;
typedef CGAL::Line_2<CGAL_KERNEL>                                   CGAL_Line_2;
typedef CGAL::Segment_2<CGAL_KERNEL>                                CGAL_Segment_2;
typedef CGAL::Polygon_with_holes_2<CGAL_KERNEL>                     CGAL_Polygon_with_holes_2;
typedef CGAL::Straight_skeleton_2<CGAL_KERNEL>                      CGAL_Ss;

typedef CGAL::Partition_traits_2<CGAL_KERNEL>                       CGAL_PT;

typedef boost::shared_ptr<CGAL_Ss>                                  CGAL_SsPtr;

typedef ClipperLib::PolyTree                                        PolyTree;
typedef ClipperLib::PolyNode                                        PolyNode;

CGAL_Polygon_2 to_cgal_polygon_2(const VectorOfVector2d &points) 
{
	CGAL_Polygon_2 poly;
	for (auto v : points)
		poly.push_back({v[0], v[1]});
	return poly;
}

// break a list of outlines into polygons with holes
std::vector<CGAL_Polygon_with_holes_2> polygons_with_holes(const Polygon2d &poly) 
{
	std::vector<CGAL_Polygon_with_holes_2> ret;
	PolyTree polytree = ClipperUtils::sanitize(ClipperUtils::fromPolygon2d(poly));  // how do we check if this was successful?

	// lambda for recursive walk through polytree
	std::function<void (PolyNode *)> walk = [&](PolyNode *c) {
		// outer path
		CGAL_Polygon_with_holes_2 c_poly(to_cgal_polygon_2(ClipperUtils::fromPath(c->Contour)));
		// holes
		for (auto cc : c->Childs) {
			c_poly.add_hole(to_cgal_polygon_2(ClipperUtils::fromPath(cc->Contour)));
			for (auto ccc : cc->Childs)
				walk(ccc);
		}
		ret.push_back(c_poly);
		return;
	};

	for (auto root_node : polytree.Childs)
		walk(root_node);

	return ret;
}

PolySet *straight_skeleton_roof(const Polygon2d &poly)
{
	PolySet *hat = new PolySet(3);

	std::vector<CGAL_Polygon_with_holes_2> shapes = polygons_with_holes(poly);
	
	for (CGAL_Polygon_with_holes_2 shape : shapes) {
		std::cout << "Computing straight skeleton..." << std::flush;
		CGAL_SsPtr ss = CGAL::create_interior_straight_skeleton_2(shape);
		std::cout << " done.\n";
		// store heights of vertices
		auto vector2d_comp = [](const Vector2d &a, const Vector2d &b) {
			return (a[0]<b[0]) || (a[0]==b[0] && a[1]<b[1]);
		};
		std::map<Vector2d,double,decltype(vector2d_comp)> heights(vector2d_comp);
		for (auto v=ss->vertices_begin(); v!=ss->vertices_end(); v++) {
			Vector2d p(v->point().x(), v->point().y());
			heights[p] = v->time();
		}

		for (auto ss_face = ss->faces_begin(); ss_face!=ss->faces_end(); ss_face++) {
			// convert ss_face to cgal polygon
			CGAL_Polygon_2 face;
			for (auto h=ss_face->halfedge(); ;) {
				CGAL_Point_2 pp = h->vertex()->point();
				face.push_back(pp);
				h = h->next();
				if (h == ss_face->halfedge()) {
					break;
				}
			}
			assert(face.is_simple());

			// do convex partition if necesary
			std::vector<CGAL_PT::Polygon_2> facets;
			CGAL::approx_convex_partition_2(face.vertices_begin(), face.vertices_end(),
					std::back_inserter(facets));

			for (auto facet : facets) {
				Polygon floor, roof;
				for (auto v=facet.vertices_begin(); v!=facet.vertices_end(); v++) {
					floor.push_back({v->x(), v->y(), 0.0});
					Vector2d vv(v->x(), v->y());
					roof.push_back({v->x(), v->y(), heights[vv]});
				}
				hat->append_poly(roof);
				std::reverse(floor.begin(), floor.end());  // floor has wrong orientation
				hat->append_poly(floor);
			}
		}
	}
	
	return hat;
}

} // roof_ss
