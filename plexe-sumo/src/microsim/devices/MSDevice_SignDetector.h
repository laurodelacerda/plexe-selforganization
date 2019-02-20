/****************************************************************************/
/// @file    MSDevice_SignDetector.h
/// @author  Lauro de Lacerda
/// @date    06.11.2018
///
// A device which detects all POI Lanes sorrouding a vehicle.
/****************************************************************************/

#ifndef MSDevice_SignDetector_h
#define MSDevice_SignDetector_h

#include <config.h>

#include "MSDevice.h"                      /// Parent class
#include <utils/common/SUMOTime.h>         /// Simulation time
#include <utils/shapes/PointOfInterest.h>  /// POIs are shapes that can be added to road networks

/// Representation of a vehicle in the road simulation
class SUMOVehicle;


class MSDevice_SignDetector : public MSDevice
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
    ~MSDevice_SignDetector();

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
        return "signDetector";
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
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_SignDetector(SUMOVehicle& holder, const std::string& id, double range, bool onlyLane);

   /** @brief Determines how a vehicle reacts to road signs
    *
    * @param[in] veh The vehicle that holds this device.
    * @param[in] poi The POI Lane that represents a road sign.
    */
    void onRoadSignDetection(SUMOVehicle& veh, PointOfInterest* poi);

private:

    /// @brief range of device
    double myRange;

    /// @brief whether device senses only from the current lane or from all lanes
    bool onlyCurrentLane;

    /// Last road sign id detected
    std::string lastRoadSignId;

    /// Last road sign type detected
    std::string lastRoadSignShape;

    /// Last lane road sign is applied
    std::string lastRoadSignLane;

    /// TODO Save index of last seen road sign in a parameter
    int lastRoadSignLaneIndex;

    // Rules that the vehicles was given to
    std::vector<std::string>* rules;

    // Current Rule
    std::string currentRule;

private:

    /// @brief Invalidated copy constructor.
    MSDevice_SignDetector(const MSDevice_SignDetector&);

    /// @brief Invalidated assignment operator.
    MSDevice_SignDetector& operator=(const MSDevice_SignDetector&);
};

#endif

/****************************************************************************/
