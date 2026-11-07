package carla;

//
// Consumer of the "carla.sensor" signal that stores every image reading
// (sensor.camera.*, rgb kind "image") as a PNG file.  Requires the emitting
// CarlaScenarioManager to have copySensorPayload = true.
//
simple CarlaImageSink
{
    parameters:
        @class(artery::carla::CarlaImageSink);

        string managerModule @desc("Path of the CarlaScenarioManager to listen to, relative to the network module. Empty listens to the whole network") = default("");
        string outputDir @desc("Directory for the captured PNG frames, relative to the run directory") = default("captures");
        int maxFrames @desc("Maximum number of image frames to write, 0 means unlimited") = default(0);
}
