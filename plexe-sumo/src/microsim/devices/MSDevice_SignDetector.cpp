/****************************************************************************/
/// @file    MSDevice_SignDetector.h
/// @author  Lauro de Lacerda
/// @date    06.11.2018
///
// A device which detects all POI Lanes around a vehicle.
/****************************************************************************/

#include "MSDevice_SignDetector.h"
#include "MSDevice_Tripinfo.h"

#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSRoute.h>
#include <utils/common/TplConvert.h>
#include <utils/common/NamedObjectCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <typeinfo>


void
MSDevice_SignDetector::insertOptions(OptionsCont& oc)
{
    oc.addOptionSubTopic("Traffic Sign Detector Device");
    insertDefaultAssignmentOptions("signDetector", "Sign Detector Device", oc);

    oc.doRegister("device.signDetector.parameter", new Option_Float(0.0));
    oc.addDescription("device.signDetector.parameter", "Sign Detector Device", "A POI detector device used to identify objects that mimic road signs.");

    oc.doRegister("device.signDetector.range", new Option_Float(0.0));
    oc.addDescription("device.signDetector.range", "Sign Detector Device", "The range in which traffic signs can be detected");

    oc.doRegister("device.signDetector.onlyCurrentLane", new Option_Bool(false));
    oc.addDescription("device.signDetector.onlyCurrentLane", "Sign Detector Device", "Specifies whether device senses traffic signs only from the current lane or from all lanes");
}


void
MSDevice_SignDetector::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into)
{
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "signDetector", v, false)) {

        double range = -1;
        if (v.getVehicleType().getParameter().knowsParameter("device.signDetector.range")) {
            try {
                range = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("device.signDetector.range", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.signDetector.range", "-1") + "'for vType parameter 'range'");
            }
        }
        else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vType parameter 'range'. Using default of " << range << "\n";
        }

        bool onlyLane = false;

        if (v.getVehicleType().getParameter().knowsParameter("device.signDetector.onlyCurrentLane")) {
            try {
                onlyLane = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.signDetector.onlyCurrentLane", "no").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.signDetector.onlyCurrentLane", "no") + "'for vType parameter 'onlyCurrentLane'");
            }
        }
        else {
            onlyLane = oc.getBool("device.signDetector.onlyCurrentLane");
        }

        MSDevice_SignDetector* device = new MSDevice_SignDetector(v, "example_" + v.getID(),
                                                        range,
                                                        onlyLane);
        into.push_back(device);
    }
}

MSDevice_SignDetector::MSDevice_SignDetector(SUMOVehicle& holder, const std::string& id, double range, bool onlyLane) :
    MSDevice(holder, id),
    myRange(range),
    onlyCurrentLane(onlyLane)
{
    std::cout << "initialized device '"   << id
              << ", range: "              << myRange
              << ", onlyCurrentLane: "    << onlyCurrentLane << "\n";

    lastRoadSignId    = "";
    lastRoadSignShape = "";
    lastRoadSignLane  = "";
    lastRoadSignLaneIndex = -1;
    rules = new std::vector<std::string>() ;
}


MSDevice_SignDetector::~MSDevice_SignDetector()
{
    delete rules;
}


bool
MSDevice_SignDetector::notifyMove(SUMOVehicle& veh, double /* oldPos */, double /* newPos */, double newSpeed)
{
    // std::cout << "device '" << getID() << "' notifyMove: newSpeed=" << newSpeed << "\n";

    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != 0)
    {
        std::cout << "Vehicle '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    }

    /// Get the current road a vehicle is passing by
    std::string currentLane = veh.getLane()->getID() ;

    MSVehicle* vehicle = dynamic_cast<MSVehicle*>(&veh);

    /// Get which lane vehicle is in to predict where vehicle will pass by
    int currentLaneIndex = vehicle->getLaneIndex();

    /// Saving the future edges and lanes of a vehicle
    std::vector<std::string> edges;
    std::vector<std::string> lanes;

    /// Maps poiId to laneIndex it is located in
    std::map<std::string, int> lanesMap;

    /// Iterating through all edges and lanes
    for (const auto& e : veh.getRoute().getEdges())
    {
        /// Edges
        if (std::find(edges.begin(), edges.end(), e->getID()) == edges.end())
            edges.push_back(e->getID());

        /// Edges' Lanes
        for (const auto& l : e->getLanes())
        {
            /// Saving in a map
            lanesMap[l->getID()] = l->getIndex();

            bool sameLane = l->getIndex() == currentLaneIndex ;

            if (onlyCurrentLane && sameLane)
                lanes.push_back(l->getID());
            else if (!onlyCurrentLane)
                lanes.push_back(l->getID());
        }
    }

    /// Get all loaded shapes
    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();

    /// Get only POIs
    for (const auto& i : shapeCont.getPOIs())
    {
        PointOfInterest* poi = dynamic_cast<PointOfInterest*>(i.second);

        // Estimating euclidian distance to traffic sign
        double distance = poi->distanceTo2D(veh.getPosition()) ;

        // Filtering POIs in the delimited range from lanes that vehicles are passing by
        if ((poi) && (std::find(lanes.begin(), lanes.end(), poi->getLane()) != lanes.end()) && (distance < myRange))
        {
            // lastRoadSignLaneIndex = lanesMap[poi->getLane()];

            // adding rules to be followed
            if (std::find(rules->begin(), rules->end(), poi->getID()) == rules->end()) {
                rules->push_back(poi->getID());
                onRoadSignDetection(veh, poi, lanesMap[poi->getLane()]);

                std::cout << "Car " << veh.getID() << " perceived road sign "
                          << poi->getID() << " - " << poi->getShapeType()
                          << " in the lane " << poi->getLane()
                          << "[" << lastRoadSignLaneIndex << "]"<< std::endl;
            }
        }
    }

    return true;
}


bool
MSDevice_SignDetector::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */)
{
    // std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


bool
MSDevice_SignDetector::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */)
{
    // std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


void
MSDevice_SignDetector::generateOutput() const
{
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
        os.openTag("SignDetector_device");
        os.writeAttr("device.signDetector.range", toString(myRange));
        os.writeAttr("device.signDetector.lastSeenSign", toString(myRange));
        os.closeTag();
    }
}

std::string
MSDevice_SignDetector::getParameter(const std::string& key) const
{
    if (key == "range") {
        return toString(myRange);
    }
    else if (key == "lastRoadSignId") {
        return lastRoadSignId;
    }
    else if (key == "lastRoadSignShape") {
        return lastRoadSignShape;
    }
    else if (key == "lastRoadSignLane"){
        return lastRoadSignLane;
    }
    else if (key == "lastRoadSignLaneIndex"){
        return toString(lastRoadSignLaneIndex);
    }
    else if (key == "range"){
        return toString(myRange);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_SignDetector::setParameter(const std::string& key, const std::string& value)
{
    double doubleValue;

    try {
        doubleValue = TplConvert::_2double(value.c_str());
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }

    if (key == "range")
        myRange = doubleValue;
    else
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}

// my functions

void
MSDevice_SignDetector::onRoadSignDetection(SUMOVehicle& veh, PointOfInterest* poi, int laneIndex)
{
  // Example of POI :
  // <poi id="POI_0" type="traffic_sign" color="white" layer="3.00" lane="2i_0"
  //  pos="169.87" imgFile="/signs/stop.png" width="2.00" height="2.00">
	// <param key="code" value="R-1"/>
	// <param key="sign_nature"    value="regulatory"/>
	// <param key="finable"        value="true"/>
	// <param key="penalty_points" value="7"/>
	// <param key="legislator"     value="police"/>
  // </poi>
    lastRoadSignId    = poi->getID();
    lastRoadSignShape = poi->getShapeType();
    lastRoadSignLane  = poi->getLane();
    lastRoadSignLaneIndex = laneIndex;
}


/****************************************************************************/
