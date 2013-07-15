#ifndef POLAR_H
#define POLAR_H

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


};


#endif // POLAR_H
