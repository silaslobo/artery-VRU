package carla;

//
// Example consumer of CARLA sensor data.  Subscribes to the "carla.sensor"
// signal of a CarlaScenarioManager and logs/aggregates the readings.
//
simple CarlaSensorSink
{
    parameters:
        @class(artery::carla::CarlaSensorSink);

        string managerModule @desc("Path of the CarlaScenarioManager to listen to, relative to the network module. Empty listens to the whole network") = default("");
        bool printReadings = default(true);
}
