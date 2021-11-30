#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    vector<Vector2D> p;
    for (int i = 0; i < points.size() - 1; i++) {
      Vector2D b0 = points[i];
      Vector2D b1 = points[i + 1];
      float x = t * b0.x + (1 - t) * b1.x;
      float y = t * b0.y + (1 - t) * b1.y;
      p.push_back(Vector2D{x, y});
    }
    return p;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    vector<Vector3D> p;
    for (int i = 0; i < points.size() - 1; i++) {
      Vector3D b0 = points[i];
      Vector3D b1 = points[i + 1];
      float x = t * b0.x + (1 - t) * b1.x;
      float y = t * b0.y + (1 - t) * b1.y;
      float z = t * b0.z + (1 - t) * b1.z;
      p.push_back(Vector3D{x, y, z});
    }
    return p;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.empty()) return NULL;
    if (points.size() == 1) return points[0];
    return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> p;
    for (int i = 0; i < controlPoints.size(); i++) {
      p.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(p, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      HalfedgeCIter h = halfedge();
      //Vector3D vPos = position;
      //vector<Vector3D> triangleVs;
      vector<Vector3D> vectorSums;
      do {
          vector<Vector3D> triangleVs;
          HalfedgeCIter hCopy = h;
          do {
              //h = h->next();
              VertexCIter v = hCopy->vertex();
              Vector3D vPos = v->position;
              triangleVs.push_back(vPos);
              hCopy = hCopy->next();
          } while (hCopy != h);
          Vector3D crossP = cross(triangleVs[1] - triangleVs[0], triangleVs[2] - triangleVs[1]);
          float magnitude = sqrt(crossP.x*crossP.x + crossP.y*crossP.y + crossP.z*crossP.z);
          float area = magnitude/2;
          Vector3D normal = cross(triangleVs[1] - triangleVs[0], triangleVs[2] - triangleVs[0]);
          vectorSums.push_back(area*normal);
          triangleVs.clear();
          HalfedgeCIter hTwin = h->twin();
          h = hTwin->next();
      } while (h != halfedge());
      
      Vector3D finalV = Vector3D(0, 0, 0);
      for (int i =0; i < vectorSums.size(); i++) {
          finalV += vectorSums[i];
      }
      finalV.normalize();
      
    return finalV;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary() || e0->halfedge()->face()->isBoundary() || e0->halfedge()->twin()->face()->isBoundary()) {
          return EdgeIter();
      }
      //HalfEdges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      //Vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      //Edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      //Faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      //FaceIter f2 = h6->face();
      //FaceIter f3 = h5->face();
      
      //FLIP!!
      //Halfedges
      h0->setNeighbors(h1, h3, v2, e0, f0);
      h1->setNeighbors(h2, h9, v3, e4, f0);
      h2->setNeighbors(h0, h6, v1, e1, f0);
      h3->setNeighbors(h4, h0, v3, e0, f1);
      h4->setNeighbors(h5, h7, v2, e2, f1);
      h5->setNeighbors(h3, h8, v0, e3, f1);
      h6->setNeighbors(h6->next(), h2, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h4, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h5, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h1, v1, e4, h9->face());
      
      //Vertices
      v0->halfedge() = h5;
      v1->halfedge() = h2;
      v2->halfedge() = h0;
      v3->halfedge() = h3;
      
      //Edges
      e0->halfedge() = h0;
      e1->halfedge() = h2;
      e2->halfedge() = h4;
      e3->halfedge() = h5;
      e4->halfedge() = h1;
      
      //Faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary() || e0->halfedge()->face()->isBoundary() || e0->halfedge()->twin()->face()->isBoundary()) {
          return VertexIter();
      }
      
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
           
           //Vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
           
           //Edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
           
           //Faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
           
           //Create New Elements
           //Vertices
      VertexIter m = newVertex();
      Vector3D v0P = v0->position, v1P = v1->position;
      Vector3D midpoint = (v0P + v1P) / 2;
      m->position = midpoint;
      m->isNew = true;
           
           //Half-edges
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();
           
           //Edges
      EdgeIter e5 = newEdge();
      //e5->isNew = true;
      EdgeIter e6 = newEdge();
      e6->isNew = true;
      EdgeIter e7 = newEdge();
      e7->isNew = true;
           
           //Faces
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
           //SPLIT!!
           //Half-edges
      h0->setNeighbors(h1, h13, v0, e0, f0);
      h1->setNeighbors(h2, h12, m, e6, f0);
      h2->setNeighbors(h0, h7, v2, e2, f0);
      h3->setNeighbors(h4, h10, v1, e5, f1);
      h4->setNeighbors(h5, h15, m, e7, f1);
      h5->setNeighbors(h3, h9, v3, e4, f1);
      h6->setNeighbors(h6->next(), h11, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h14, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
      h10->setNeighbors(h11, h3, m, e5, f2);
      h11->setNeighbors(h12, h6, v1, e1, f2);
      h12->setNeighbors(h10, h1, v2, e6, f2);
      h13->setNeighbors(h14, h0, m, e0, f3);
      h14->setNeighbors(h15, h8, v0, e3, f3);
      h15->setNeighbors(h13, h4, v3, e7, f3);
           
           //Vertices
      v0->halfedge() = h0;
      v1->halfedge() = h3;
      v2->halfedge() = h12;
      v3->halfedge() = h15;
      m->halfedge() = h10;
           
           //Edges
      e0->halfedge() = h0;
      e1->halfedge() = h11;
      //e2->halfedge() = h12;
      e2->halfedge() = h2;
      e3->halfedge() = h14;
      e4->halfedge() = h5;
      e5->halfedge() = h3;
      e6->halfedge() = h1;
      e7->halfedge() = h4;
        
           //Faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h10;
      f3->halfedge() = h13;
      
      
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.
     for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          HalfedgeIter currH = v->halfedge();
          
          float n = v->degree();
          float u;
          if (n == 3) {
              u = 3.0/16.0;
          } else {
              float bottom = 8 * n;
              u = 3.0/bottom;
          }
          
          Vector3D originalPos = v->position;
          Vector3D originalNextSum;
          HalfedgeIter h = currH;
          
          do {
              HalfedgeIter h_twin = h->twin();
              VertexIter ver = h_twin->vertex();
              originalNextSum += ver->position;
              
              EdgeIter e = h_twin->edge();
              if (!e->isBoundary()) {
                  VertexIter v0 = h->vertex();
                  VertexIter v1 = h->twin()->vertex();
                  VertexIter v2 = h->next()->next()->vertex();
                  VertexIter v3 = h->twin()->next()->next()->vertex();
                  //changing to test butterfly
                  /*VertexIter v4 = h->next()->next()->twin()->next()->next()->vertex();
                  VertexIter v5 = h->twin()->next()->twin()->next()->next()->vertex();
                  VertexIter v6 = h->twin()->next()->next()->twin()->next()->next()->vertex();
                  VertexIter v7 = h->next()->twin()->next()->next()->vertex();*/
                  //
                  Vector3D v0p = v0->position;
                  Vector3D v1p = v1->position;
                  Vector3D v2p = v2->position;
                  Vector3D v3p = v3->position;
                  //added
                  /*
                  Vector3D v4p = v4->position;
                  Vector3D v5p = v5->position;
                  Vector3D v6p = v6->position;
                  Vector3D v7p = v7->position;*/
                  
                  Vector3D eNewPos = ((3.0/8.0) * (v0p + v1p)) + ((1.0/8.0) * (v2p + v3p)); // - ((1.0/16.0) * (v4p + v5p + v6p + v7p));
                  
                  e->newPosition = eNewPos;
                  
              }
              
              h = h_twin->next();
          } while (h != currH);
          
          Vector3D newPos = ((1 - (n * u)) * originalPos) + (u * originalNextSum);
          
          v->newPosition = newPos;
          v->newPosition = newPos;
          
      }
      //PartB
      
      EdgeIter e = mesh.edgesBegin();
      
      while (e != mesh.edgesEnd()) {
          EdgeIter nextEdge = e;
          nextEdge++;
          //split
          HalfedgeIter h1 = e->halfedge();
          HalfedgeIter h2 = h1->twin();
          VertexIter v0 = h1->vertex();
          VertexIter v1 = h2->vertex();
          if (!e->isBoundary() && !(v0->isNew || v1->isNew)) {
              VertexIter m = mesh.splitEdge(e);
              m->isNew = true;
              m->newPosition = e->newPosition;
              v0->isNew = false;
              m->halfedge()->twin()->vertex()->isNew = false;
              
              v0->halfedge()->edge()->isNew = false;
              m->halfedge()->edge()->isNew = false;
              
          }
          e = nextEdge;
          
      }
       
      e = mesh.edgesBegin();
      
      while (e != mesh.edgesEnd()) {
          EdgeIter nextEdge = e;
          nextEdge++;
          //flip
          VertexIter v0 = e->halfedge()->vertex();
          VertexIter v1 = e->halfedge()->twin()->vertex();
          if ( (e->isNew) && ((v0->isNew && !v1->isNew) || (!v0->isNew && v1->isNew)) ) {
              mesh.flipEdge(e);
              
              e->isNew = false;
          }
         
          e = nextEdge;
      }
      //Part C
      for (VertexIter v = mesh.verticesBegin(); v!= mesh.verticesEnd(); v++) {
          Vector3D newPos = v->newPosition;
          v->position = newPos;
          v->isNew = false;
      }
  }
}

