#ifndef G2O_EDGE_SE3_H_
#define G2O_EDGE_SE3_H_

#include "g2o/core/base_binary_edge.h"

#include "g2o_types_slam3d_api.h"
#include "vertex_se3.h"

namespace g2o {

  /**
   * \brief Edge between two 3D pose vertices
   *
   * The transformation between the two vertices is given as an Isometry3D.
   * If z denotes the measurement, then the error function is given as follows:
   * z^-1 * (x_i^-1 * x_j)
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3 : public BaseBinaryEdge<6, Isometry3D, VertexSE3, VertexSE3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
      // virtual bool read_1(std::istream& is);

      bool extraSetEdge_Adjacent(double* data) {
        Vector7d meas;
        for (int i=0; i<7; i++) 
          meas[i] = data[i];
        // normalize the quaternion to recover numerical precision lost by storing as human readable text
        Vector4D::MapType(meas.data()+3).normalize();
        setMeasurement(internal::fromVectorQT(meas));

        std::cout << "load infor......" << std::endl;
        int index = 0;
        double infor[] = {10, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 50, 0, 0, 50, 0, 50};
        for ( int i=0; i<information().rows() && index<21; i++)
          for (int j=i; j<information().cols(); j++){
            double tmp_infor = infor[index];
            index++;
            if (i!=j)
              information()(j,i) = tmp_infor;
          }

        return true;
      }
      
      bool read_information( /* double* data, */ std::istream& is) {
        std::cout << "lgw: g2o....." << std::endl;
        Vector7d meas;
        for (int i=0; i<7; i++){
          is >> meas[i] ;
          // meas[i] = data[i];
        }
        // normalize the quaternion to recover numerical precision lost by storing as human readable text
        Vector4D::MapType(meas.data()+3).normalize();
        setMeasurement(internal::fromVectorQT(meas));

        if (is.bad()) {
          return false;
        }
        for ( int i=0; i<information().rows() && is.good(); i++)
          for (int j=i; j<information().cols() && is.good(); j++){
            is >> information()(i,j);
            if (i!=j)
              information()(j,i)=information()(i,j);
          }
        if (is.bad()) {
          //  we overwrite the information matrix with the Identity
          information().setIdentity();
        } 
        return true;
      }

      bool extraSetEdge_loop(double* data) {
        Vector7d meas;
        for (int i=0; i<7; i++) 
          meas[i] = data[i];
        // normalize the quaternion to recover numerical precision lost by storing as human readable text
        Vector4D::MapType(meas.data()+3).normalize();
        setMeasurement(internal::fromVectorQT(meas));

        int index = 0;
        double infor[] = {1000, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 1000, 0, 0, 1000, 0, 1000};
        for ( int i=0; i<information().rows() && index<21; i++)
          for (int j=i; j<information().cols(); j++){
            double tmp_infor = infor[index];
            index++;
            if (i!=j)
              information()(j,i) = tmp_infor;
          }

        return true;
      }

      void computeError();

      virtual void setMeasurement(const Isometry3D& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector7d> v(d);
        setMeasurement(internal::fromVectorQT(v));
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector7d> v(d);
        v = internal::toVectorQT(_measurement);
        return true;
      }

      void linearizeOplus();

      virtual int measurementDimension() const {return 7;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      Isometry3D _inverseMeasurement;
  };

  /**
   * \brief Output the pose-pose constraint to Gnuplot data file
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  /**
   * \brief Visualize a 3D pose-pose constraint
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3DrawAction: public DrawAction{
  public:
    EdgeSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace
#endif
