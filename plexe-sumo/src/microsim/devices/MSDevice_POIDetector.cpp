/****************************************************************************/
/// @file    MSDevice_POIDetector.h
/// @author  Lauro de Lacerda
/// @date    06.11.2018
///
// A device which detects all POI Lanes around a vehicle.
/****************************************************************************/

#include "MSDevice_POIDetector.h"
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
MSDevice_POIDetector::insertOptions(OptionsCont& oc)
{
    oc.addOptionSubTopic("POI Detector Device");
    insertDefaultAssignmentOptions("poiDetector", "POI Detector Device", oc);

    oc.doRegister("device.poiDetector.parameter", new Option_Float(0.0));
    oc.addDescription("device.poiDetector.parameter", "POI Detector Device", "A POI detector device used to identify Point Of Interest objects");

    oc.doRegister("device.poiDetector.range", new Option_Float(0.0));
    oc.addDescription("device.poiDetector.range", "POI Detector Device", "The range in which POIs can be detected");

    oc.doRegister("device.poiDetector.onlyPOIOutside", new Option_Bool(false));
    oc.addDescription("device.poiDetector.onlyPOIOutside", "POI Detector Device", "Specifies whether device detects only POIs not attached to lanes");

    oc.doRegister("device.poiDetector.onlyPOILane", new Option_Bool(false));
    oc.addDescription("device.poiDetector.onlyPOILane", "POI Detector Device", "Specifies whether a device detects only POIs attached to lanes");

    oc.doRegister("device.poiDetector.onlyCurrentLane", new Option_Bool(false));
    oc.addDescription("device.poiDetector.onlyCurrentLane", "POI Detector Device", "Specifies whether a device detects only POIs from the current lane they are travelling");
}


void
MSDevice_POIDetector::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into)
{
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "poiDetector", v, false)) 
    {

        double range = -1;
        if (v.getVehicleType().getParameter().knowsParameter("device.poiDetector.range")) {
            try 
            {
                range = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("device.poiDetector.range", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.poiDetector.range", "-1") + "'for vType parameter 'range'");
            }
        }
        else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vType parameter 'range'. Using default of " << range << "\n";
        }

        bool poiOut = false;

        if (v.getVehicleType().getParameter().knowsParameter("device.poiDetector.onlyPOIOutside"))
        {   
            try 
            {
                poiOut = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyPOIOutside", "no").c_str());
            } 
            catch (...) 
            {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyPOIOutside", "no") + "'for vType parameter 'onlyPOIOutside'");
            }
        }
        else 
        {
            poiOut = oc.getBool("device.poiDetector.onlyPOIOutside");
        }

        bool poiLane = false;

        if (v.getVehicleType().getParameter().knowsParameter("device.poiDetector.onlyPOILane")) 
        {
            try 
            {
                poiLane = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyPOILane", "no").c_str());
            } 
            catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyPOILane", "no") + "'for vType parameter 'onlyPOILane'");
            }
        }
        else 
        {
            poiLane = oc.getBool("device.poiDetector.onlyPOILane");
        }

        bool onlyLane = false;

        if (v.getVehicleType().getParameter().knowsParameter("device.poiDetector.onlyCurrentLane")) 
        {
            try 
            {
                onlyLane = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyCurrentLane", "no").c_str());
            } 
            catch (...) 
            {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.poiDetector.onlyCurrentLane", "no") + "'for vType parameter 'onlyCurrentLane'");
            }
        }
        else {
            onlyLane = oc.getBool("device.poiDetector.onlyCurrentLane");
        }

        MSDevice_POIDetector* device = new MSDevice_POIDetector(v, 
                                                                "example_" + v.getID(),
                                                                range,
                                                                poiOut,
                                                                poiLane,
                                                                onlyLane);
        into.push_back(device);
    }
}

MSDevice_POIDetector::MSDevice_POIDetector(SUMOVehicle& holder, const std::string& id, double range, bool poiOut, bool poiLane, bool onlyLane) :
    MSDevice(holder, id),
    detectionRange(range),
    onlyPOIOutside(poiOut),
    onlyPOILane(poiLane),
    onlyCurrentLane(onlyLane)
{
    std::cout << "initialized device '"   << id
              << ", range: "              << detectionRange
              << ", onlyPOIOutside:"      << onlyPOIOutside
              << ", onlyPOILane:"         << onlyPOILane
              << ", onlyCurrentLane: "    << onlyCurrentLane << "\n";

    lastPOIId    = "";
    lastPOIShape = "";
    lastPOILane  = "";
    lastPOILaneIndex = -1;
    pois = new std::vector<std::string>() ;
}


MSDevice_POIDetector::~MSDevice_POIDetector()
{
    delete pois;
}


bool
MSDevice_POIDetector::notifyMove(SUMOVehicle& veh, double /* oldPos */, double /* newPos */, double newSpeed)
{
    // std::cout << "device '" << getID() << "' notifyMove: newSpeed=" << newSpeed << "\n";

    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != 0)
    {
        std::cout << "Vehicle '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    }

    /// Get the current lane a vehicle is passing by
    std::string currentLane = veh.getLane()->getID() ;

    MSVehicle* vehicle = dynamic_cast<MSVehicle*>(&veh);

    /// Get which lane vehicle is in to predict where vehicle will pass by
    int currentLaneIndex = vehicle->getLaneIndex();

    /// Saving the future edges and lanes of a vehicle
    std::vector<std::string> edges;
    std::vector<std::string> lanes;

    /// Maps poiId to laneIndex it is located in
    std::map<std::string, int> poiLanesMap;

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
            poiLanesMap[l->getID()] = l->getIndex();

            bool sameLane = l->getIndex() == currentLaneIndex ;

            if (onlyCurrentLane and sameLane)
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
        if ((poi) and (distance < detectionRange))            
        {        
            // POIs from Lanes // onlyPOIOutside onlyPOILane onlyCurrentLane

            // It's a new POI
            bool newPOI = std::find(pois->begin(), pois->end(), poi->getID()) == pois->end();
            
            // It's a new POI Lane
            bool inLane = std::find(lanes.begin(), lanes.end(), poi->getLane()) != lanes.end();

            bool onlyPOIOut   =  onlyPOIOutside and !onlyPOILane;
            bool allPOIs      = !onlyPOIOutside and !onlyPOILane;
            bool allPOILanes  = !onlyPOIOutside and  onlyPOILane and !onlyCurrentLane;
            bool onlyPOICLane = !onlyPOIOutside and  onlyPOILane and  onlyCurrentLane;
            
            if(newPOI)
            {
                if(allPOILanes and inLane)
                    onPOIDetected(veh, poi, poiLanesMap[poi->getLane()]);
                else if (allPOIs)
                    onPOIDetected(veh, poi, poiLanesMap[poi->getLane()]);
            }
        }
    }

    return true;
}


bool
MSDevice_POIDetector::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */)
{
    // std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


bool
MSDevice_POIDetector::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */)
{
    // std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


void
MSDevice_POIDetector::generateOutput() const
{
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
        os.openTag("SignDetector_device");
        os.writeAttr("device.poiDetector.range", toString(detectionRange));
        os.writeAttr("device.poiDetector.lastSeenSign", toString(detectionRange));
        os.closeTag();
    }
}

std::string
MSDevice_POIDetector::getParameter(const std::string& key) const
{
    if (key == "range") {
        return toString(detectionRange);
    }
    else if (key == "lastPOIId") {
        return lastPOIId;
    }
    else if (key == "lastPOIShape") {
        return lastPOIShape;
    }
    else if (key == "lastPOILane"){
        return lastPOILane;
    }
    else if (key == "lastPOILaneIndex"){
        return toString(lastPOILaneIndex);
    }
    else if (key == "range"){
        return toString(detectionRange);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_POIDetector::setParameter(const std::string& key, const std::string& value)
{
    double doubleValue;

    try {
        doubleValue = TplConvert::_2double(value.c_str());
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }

    if (key == "range")
        detectionRange = doubleValue;
    else
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_POIDetector::onPOIDetected(SUMOVehicle& veh, PointOfInterest* poi, int laneIndex)
{
    pois->push_back(poi->getID());

    lastPOIId        = poi->getID();
    lastPOIShape     = poi->getShapeType();
    lastPOILane      = poi->getLane();    
    lastPOILaneIndex = laneIndex;

    std::cout << "Car " << veh.getID() << " detected POI " << poi->getID() << " - " << poi->getShapeType()
    << " in the lane " << poi->getLane() << "[" << lastPOILaneIndex << "]"<< std::endl;
}
