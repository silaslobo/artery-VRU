#include "artery/application/ClusterManagement.h"
#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include <vanetza/asn1/vam.hpp>
#include "artery/application/Constants.hpp"
#include <omnetpp/simtime.h>
#include <string>
#include "artery/application/PersonDataProvider.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include "artery/traci/PersonController.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>
#include "traci/API.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "traci/Boundary.h"
#include "artery/traci/Cast.h"
#include "traci/Core.h"

namespace artery
{
namespace bg = boost::geometry;
using namespace omnetpp;

//Define_Module(ClusterManagement)

ClusterManagement::ClusterManagement(){}

// ClusterManagement::ClusterManagement(const PersonDataProvider* pdp, 
// 									artery::LocalDynamicMap* ldm, 
// 									const traci::PersonController* pc, 
// 									VruRoleType initialVruRole) : 
// 									mPersonDataProvider(pdp), 
// 									mLocalDynamicMap(ldm), 
// 									mPersonController(pc), 
// 									currentVruRole(initialVruRole){} 

void ClusterManagement::initialize(ClusterManagementParameters_t& cManagementPar){

	//Clustering options and parameters
	mNumCreateCluster = cManagementPar.NumCreateCluster;
	mMaxClusterDistance = cManagementPar.MaxClusterDistance;
	mMaxClusterVelocityDifference = cManagementPar.MaxClusterVelocityDifference;
	mMaxCombinedClusterDistance = cManagementPar.MaxCombinedClusterDistance;
	mMinClusterSize = cManagementPar.MinClusterSize;
	mMaxClusterSize = cManagementPar.MaxClusterSize;
	mNumClusterVAMRepeat = cManagementPar.NumClusterVAMRepeat;

	// Clustering timing parameters
	mTimeClusterUniquenessThreshold = cManagementPar.TimeClusterUniquenessThreshold;
	mTimeClusterBreakupWarning = cManagementPar.TimeClusterBreakupWarning;
	mTimeClusterJoinNotification = cManagementPar.TimeClusterJoinNotification;
	mTimeClusterJoinSuccess = cManagementPar.TimeClusterJoinSuccess;
	mTimeClusterIdChangeNotification = cManagementPar.TimeClusterIdChangeNotification;
	mTimeClusterIdPersist = cManagementPar.TimeClusterIdPersist;
	mTimeClusterContinuity = cManagementPar.TimeClusterContinuity;
	mTimeClusterLeaderWaiting = cManagementPar.TimeClusterLeaderWaiting;
	mTimeClusterLeaveNotification = cManagementPar.TimeClusterLeaveNotification;
	mTimeCombinedVruClusterOpportunity = cManagementPar.TimeCombinedVruClusterOpportunity; 
	
	mBoundingBoxShape = cManagementPar.DefaultBoundingBoxShape;

	setCurrentVruRole(VruActiveStandalone);
}

void ClusterManagement::simObjects(const PersonDataProvider* pdp, artery::LocalDynamicMap* ldm, const traci::PersonController* pc){

	mPersonDataProvider =  pdp;
	mLocalDynamicMap = ldm;
	mPersonController = pc;

	api = mPersonController->getTraCI();
}

void ClusterManagement::setCurrentVruRole(VruRoleType newVruRole){currentVruRole = newVruRole;}

ClusterManagement::VruRoleType ClusterManagement::getCurrentVruRole(){return currentVruRole;}

bool ClusterManagement::isVruIdle(){return currentVruRole == VruIdle;}
bool ClusterManagement::isVruStandalone(){return currentVruRole == VruActiveStandalone;}
bool ClusterManagement::isVruClusterLeader(){return currentVruRole == VruActiveClusterLeader;}
bool ClusterManagement::isVruPassive(){return currentVruRole == VruPassive;}


bool ClusterManagement::vruShouldJoin(const vanetza::asn1::Vam& vam){
	ClusterParameters_t clusterPar = parseInformationContainer(vam);
	return checkClusteringConditions(clusterPar.distance, clusterPar.velocityDifference, clusterPar.cardinality, clusterPar.withinBoundingBox);
}

bool ClusterManagement::vruShouldLeave(const vanetza::asn1::Vam& vam){
	ClusterParameters_t clusterPar = parseInformationContainer(vam);
	return !checkClusteringConditions(clusterPar.distance, clusterPar.velocityDifference, clusterPar.cardinality, clusterPar.withinBoundingBox);
}

bool ClusterManagement::vruShouldCreateCluster(){
	// TODO: Should also evaluate if any available leader is nearby -> parse a set of "recent" VAMs 
	return (mVruClusterMap.size() + 1) >= mNumCreateCluster;
}

artery::ClusterManagement::ClusterParameters_t ClusterManagement::parseInformationContainer(const vanetza::asn1::Vam& vam){

	VruAwareness_t vam_data = (*vam).vam;
	ClusterParameters_t clusterParameters;

	clusterParameters.cardinality = vam_data.vamParameters.vruClusterInformationContainer->clusterCardinalitySize;

	double vruLeaderLon = vam_data.vamParameters.basicContainer.referencePosition.longitude/std::pow(10.0, 7);
	double vruLeaderLat = vam_data.vamParameters.basicContainer.referencePosition.latitude/std::pow(10.0, 7);

	libsumo::TraCIPosition leaderTraCIPos = api->simulation.convertGeo(vruLeaderLon,vruLeaderLat, true);
	pointXY leaderPos = pointXY(leaderTraCIPos.x,leaderTraCIPos.y);

	libsumo::TraCIPosition vruTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
	pointXY vruPos = pointXY(vruTraCIPos.x, vruTraCIPos.y);

	clusterParameters.distance = bg::distance(vruPos, leaderPos)*vanetza::units::si::meter;

	SpeedValue_t vruLeaderSpeed = vam_data.vamParameters.vruHighFrequencyContainer->speed.speedValue;
	SpeedValue_t vruSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());

	clusterParameters.velocityDifference = velocityDifference(vruLeaderSpeed,vruSpeed);

	if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterPolygon){
		PolyPointList_t& receivedOffsetList = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList;

		Polygon_t receivedBoundingBox = reconstructPolygon(receivedOffsetList,leaderTraCIPos);
		clusterParameters.withinBoundingBox = bg::within(vruPos, receivedBoundingBox);

	}else if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterCircle){
		Circle_t receivedBoundingBox = buildCircle(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.radius, leaderPos);
		clusterParameters.withinBoundingBox = bg::within(vruPos, receivedBoundingBox);
	}

	return clusterParameters;
	
}

// Reconstructs Polygon Bounding Box based on offset list
artery::ClusterManagement::Polygon_t ClusterManagement::reconstructPolygon(const PolyPointList_t& receivedOffsetList, libsumo::TraCIPosition& leaderPos)
{
	std::vector<pointXY> reconstructedPositions;

	std::map<int,OffsetPoint_t*> offsetMap;
	for(int iter = 0; iter < receivedOffsetList.list.count; iter++){
		offsetMap.insert(std::pair<int, OffsetPoint_t*>(iter, receivedOffsetList.list.array[iter]));
	}

	bool firstIteration = true;
	pointXY previousPos;
	for(auto& offsetPoint : offsetMap){

		if(firstIteration){
			previousPos = pointXY(leaderPos.x, leaderPos.y);
			firstIteration = false;
		}

		Node_XY_22b_t* node = &offsetPoint.second->nodeOffsetPointxy.choice.node_XY2;
		pointXY currentPos = pointXY(bg::get<0>(previousPos) + node->x/100, bg::get<1>(previousPos) + node->y/100);
		reconstructedPositions.push_back(currentPos);
		previousPos = currentPos;
	}

	Polygon_t reconstructedPolygon;
	bg::assign_points(reconstructedPolygon, reconstructedPositions);
	return reconstructedPolygon;
}

artery::ClusterManagement::Circle_t ClusterManagement::buildCircle(const double radius, pointXY& leaderPos)
{
	Circle_t builtCircle;
	bg::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
	bg::strategy::buffer::point_circle circle_strategy(360);
	bg::strategy::buffer::join_round join_strategy;
	bg::strategy::buffer::end_round end_strategy;
	bg::strategy::buffer::side_straight side_strategy;

	bg::buffer(leaderPos, builtCircle, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
	return builtCircle;
}

std::vector<std::tuple<long,long>> ClusterManagement::clusterPoly(const std::map<LocalDynamicMap::StationID, artery::VaObject> vruMap)
{
	std::map<LocalDynamicMap::StationID, artery::VaObject> clusterMap = vruMap;
	std::vector<pointXY> clusterPositions;
	pointXY previousPos, vruEgoPos;
	std::vector<std::tuple<long,long>> distancesList;

	libsumo::TraCIPosition vruEgoTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
	vruEgoPos = pointXY(vruEgoTraCIPos.x, vruEgoTraCIPos.y);
	clusterPositions.push_back(vruEgoPos);

	//egoHeading
	//auto egoVruHeading = Constants::round(mPersonDataProvider->heading(), Constants::decidegree);	

	for (auto const& itr : clusterMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		clusterPositions.push_back(pointXY(vruTraCIPos.x, vruTraCIPos.y));			
	}

	// Adds leader node again to close polygon
	clusterPositions.push_back(vruEgoPos);

	// Using boost geometry, simplifies the original polygon with convex hull algorithm
	Polygon_t clusterPolygon, hull;
	bg::assign_points(clusterPolygon, clusterPositions);

	std::cout << "Original poly: " << bg::dsv(clusterPolygon) << std::endl;

	bg::convex_hull(clusterPolygon, hull);

	std::cout << "Hull poly: " << bg::dsv(hull) << std::endl;

	// replaces original vertices with simplified polygon from convex hull
	clusterPositions = hull.outer();

	// Obtains list of offsets
	bool firstIteration = true;
	for (auto const& currentPos : clusterPositions){

		long deltaX , deltaY;
		
		// First value is obtained from the offset between vruEgo and starting point of the polygon
		// If the starting point is the vruEgo, a null first offset is expected
		if(firstIteration){
			firstIteration = false;
			previousPos = vruEgoPos;
			std::cout << "offset list leader: ";
		}

		// Offset in centimeters assuming a cartesian plane
		deltaX = std::round(bg::get<0>(currentPos) - bg::get<0>(previousPos))*100; 
		deltaY = std::round(bg::get<1>(currentPos) - bg::get<1>(previousPos))*100;

		std::cout << "(" << std::to_string(deltaX) << "," << std::to_string(deltaY) << "),";

		distancesList.push_back(std::make_tuple(deltaX, deltaY));
		previousPos = currentPos;
	}
	std::cout << std::endl;

	return distancesList;
}

double ClusterManagement::velocityDifference(SpeedValue_t referenceSpeed, SpeedValue_t evaluatedSpeed)
{
	return (referenceSpeed > 0) ? std::abs(static_cast<double>(referenceSpeed - evaluatedSpeed))/referenceSpeed : 0.0;
}

bool ClusterManagement::checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox)
{
	return (cardinality < mMaxClusterSize && velocityDifference <= mMaxClusterVelocityDifference && (distance <= mMaxClusterDistance || withinBoundingBox));
}

void ClusterManagement::createVruClusterMap(){
	
	mVruClusterMap.clear();

	std::vector<double> mNearbyVruDistances;

	auto lDM = mLocalDynamicMap->localMap();
			
	//calculate distance between egoVRU and received VRU
	double VruEgoLon = ((Constants::round(mPersonDataProvider->longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast) / std::pow( 10.0, 7 ));
	double VruEgoLat = ((Constants::round(mPersonDataProvider->latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth) / std::pow( 10.0, 7 )); 
	libsumo::TraCIPosition vruEgoTraCIPos = api->simulation.convertGeo(VruEgoLon,VruEgoLat, true);
	SpeedValue_t VruEgoSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());

	//collect data from the VRUs in the LDM and update mVruClusterMap		
	for (auto const& itr : *lDM) {
									
		double mVlatitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));				
		double mVlongitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));			
		libsumo::TraCIPosition mVTraCIPosition = api->simulation.convertGeo(mVlongitude, mVlatitude, true);
				
		vanetza::units::Length distanceVRU = bg::distance(pointXY(vruEgoTraCIPos.x,vruEgoTraCIPos.y), pointXY(mVTraCIPosition.x,mVTraCIPosition.y)) * vanetza::units::si::meter;	

		SpeedValue_t mVSpeed = itr.second.object.asn1()->vam.vamParameters.vruHighFrequencyContainer->speed.speedValue;
		double clusterVelocityDifference = velocityDifference(VruEgoSpeed, mVSpeed);

		//update mVruClusterMap when distance, velocity and cardinality constraints are met		
		if(checkClusteringConditions(distanceVRU,clusterVelocityDifference,mVruClusterMap.size()+1) && !itr.second.object.asn1()->vam.vamParameters.vruClusterInformationContainer) {	

			mNearbyVruDistances.push_back(distanceVRU/vanetza::units::si::meter);

			uint32_t stationID = itr.second.object.asn1()->header.stationID;
			artery::VaObject awr = itr.second.object;
			mVruClusterMap.insert(std::pair<LocalDynamicMap::StationID, artery::VaObject>(stationID, awr));
		}
	}

	if(mNearbyVruDistances.size() > 0){
		mSmallestDistanceToVru = *std::min_element(mNearbyVruDistances.begin(), mNearbyVruDistances.end());
	}

}

void ClusterManagement::updateVruClusterMap(){

	// Should only update the content when there are valid join or leave operations

}

}