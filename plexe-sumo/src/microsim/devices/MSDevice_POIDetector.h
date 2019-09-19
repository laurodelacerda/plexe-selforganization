/****************************************************************************/
/// @file    MSDevice_POIDetector.h
/// @author  Lauro de Lacerda
/// @date    06.11.2018
///
// A device which detects all POIs sorrouding a vehicle.
/****************************************************************************/

#ifndef MSDevice_POIDetector_h
#define MSDevice_POIDetector_h

#include <config.h>

#include "MSDevice.h"                      /// Parent class
#include <utils/common/SUMOTime.h>         /// Simulation time
#include <utils/shapes/PointOfInterest.h>  /// POIs are shapes that can be added to road networks

/// Representation of a vehicle in the road simulation
class SUMOVehicle;


class MSDevice_POIDetector : public MSDevice
{

public:

    /** @brief Inserts parameters into to configure device
     *  @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build other devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a example-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in]     v    The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into);


public:

    /// @brief Destructor.
    ~MSDevice_POIDetector();

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh      Vehicle that asks this reminder.
     * @param[in] oldPos   Position before move.
     * @param[in] newPos   Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     *
     * @return True (always).
     */
    bool notifyMove(SUMOVehicle& veh, double oldPos, double newPos, double newSpeed);

    /** @brief Saves departure info on insertion
     *
     * @param[in]   veh The entering vehicle.
     * @param[in]   reason how the vehicle enters the lane
     * @return Always true
     */
    bool notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);

    /** @brief Saves arrival info
     *
     * @param[in] veh           The leaving vehicle.
     * @param[in] lastPos       Position on the lane when leaving.
     * @param[in] isArrival     whether the vehicle arrived at its destination
     * @param[in] isLaneChange  whether the vehicle changed from the lane
     * @return True if it did not leave the net.
     */
    bool notifyLeave(SUMOVehicle& veh, double lastPos, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);


    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "poiDetector";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::generateOutput
     */
    void generateOutput() const;


private:

    /** @brief Constructor
     *
     * @param[in] holder    The vehicle that holds this device
     * @param[in] id        The ID of the device
     * @param[in] range     Range of POI to detect
     * @param[in] poiOut    Whether should detect only POIs not attached to lanes
     * @param[in] poiLane   Whether should detect only POIs attached to lanes
     * @param[in] onlyLane  Whether should detect only POIs from the current lane they are travelling
     */
    MSDevice_POIDetector(SUMOVehicle& holder, const std::string& id, double range, bool poiOut, bool poiLane, bool onlyLane);

   /** @brief Determines how a vehicle reacts to a POI
    *
    * @param[in] veh        The vehicle that holds this device.
    * @param[in] poi        The POI 
    * @param[in] laneIndex  The lane index the vehicles are in
    */
    void onPOIDetected(SUMOVehicle& veh, PointOfInterest* poi, int laneIndex = -1);

private:

    /// @brief range of device
    double detectionRange;

    /// @brief whether device detects only POIs not attached to lanes
    bool onlyPOIOutside;

    /// @brief whether device detects only POIs attached to lanes
    bool onlyPOILane;

    /// @brief whether device detects only POIs from the current lane they are travelling
    bool onlyCurrentLane;

    /// Last POI id
    std::string lastPOIId;

    /// Last POI type
    std::string lastPOIShape;

    /// Last POI lane
    std::string lastPOILane;

    /// Last POI lane index
    int lastPOILaneIndex;

    // POIs detected
    std::vector<std::string>* pois;

private:

    /// @brief Invalidated copy constructor.
    MSDevice_POIDetector(const MSDevice_POIDetector&);

    /// @brief Invalidated assignment operator.
    MSDevice_POIDetector& operator=(const MSDevice_POIDetector&);
};

#endif

/****************************************************************************/
