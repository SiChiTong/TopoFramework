#ifndef POLAR_H
#define POLAR_H

#include "Vector3.h"
#include <cmath>
/**
 * This class implements a structure for keeping polar coordinates.
 */
class Polar{

 public:

    /** elevation -> height angle (from horizon) [-90;..;+90]
     *  azimuth   -> planar angle (from positive x-axis) 
     *  distance  -> distance from object */
    float distance;
    float azimuth;
    float elevation;

    /** Standard constructor */
    Polar(void) :
      distance(0.f),
      azimuth(0.f),
      elevation(0.f)
    {/* Nothing to do! */ }

    /** Parameter Constructor */
    Polar(const float distance,
	        const float azimuth,
	        const float elevation) :
	          distance(distance),
	          azimuth(azimuth),
	          elevation(elevation)
    {/* Nothing to do! */ }

    /** Vector3 constuctor */
    //Polar(const Vector3<float>& vector);
    //Polar(const Vector3<double>& vector);

    /** Copy constructor */
    Polar(const Polar& polar)
        : distance(polar.distance),
          azimuth(polar.azimuth),
          elevation(polar.elevation)
    { /* Nothing to do! */ }

    /** Destructor */
    //virtual ~Polar(void); // @@>>TODO: streaming problem?

    /** Assignment */
    inline Polar& operator=(const Polar& polar){
        this->distance  = polar.distance;
        this->azimuth   = polar.azimuth;
        this->elevation = polar.elevation;
        return *this;
    }

    /**
     * Two polar instances are equal when rho, phi and theta are equal
     * @author Tobias Warden
     * @param polar the polar instance for comparison
     */
    inline bool operator==(const Polar& polar){
        return (this->distance  == polar.distance &&
                this->azimuth   == polar.azimuth  &&
                this->elevation == polar.elevation);
    }

    /**
     * This function is implemented as the exact reverse of the equals
     * operator.
     * @author Tobias Warden
     * @param polar the polar instance for comparison
     */
    inline bool operator!=(const Polar& polar){
        return !(operator==(polar));
    }

    // Convert this polar to Vector3 representation.
    inline Vector3<double> toVector() const
    {
      Vector3<double> vec;
      vec.x = distance * std::cos(elevation * M_PI / 180.0) * std::cos(azimuth * M_PI / 180.0);
      vec.y = distance * std::cos(elevation * M_PI / 180.0) * std::sin(azimuth * M_PI / 180.0);
      vec.z = distance * std::sin(elevation * M_PI / 180.0);
      return vec;
    }
};


#endif // POLAR_H
