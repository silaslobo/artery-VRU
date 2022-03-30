import storyboard
import timeline
from storyboard import Coord


def createStories(board):
        # Create TtcCondition
        ttcCondition = storyboard.TtcCondition(1.5, 200)  # ttc 1.5sec | radius 200m
        speedConditonGreater = storyboard.SpeedDifferenceConditionFaster(5.55556)  # 20 km/h
        ttcSignalEffect = storyboard.SignalEffect("irc")
        speedAndTTC = storyboard.AndCondition(ttcCondition, speedConditonGreater)

        story = storyboard.Story(speedAndTTC, [ttcSignalEffect])

        # Vehicle stops at second 20
        timeToStop = storyboard.TimeCondition(timeline.seconds(40.5))
        vehicleToStop = storyboard.CarSetCondition("flow1.0")
        vehicleAndTimeToStop = storyboard.AndCondition(timeToStop, vehicleToStop)
        stopEffect = storyboard.StopEffect()

        stopStory = storyboard.Story(vehicleAndTimeToStop, [stopEffect])

        # Register Stories at the Storyboard
        board.registerStory(story)
        board.registerStory(stopStory)

        jamAreaCondition = storyboard.PolygonCondition([Coord(2920, 2673),
                                                        Coord(2903, 2955),
                                                        Coord(2882, 3154),
                                                        Coord(2901, 3154),
                                                        Coord(2923, 2955),
                                                        Coord(2943, 2673)])
        jamTimeCondition = storyboard.TimeCondition(timeline.seconds(60))
        jamTimeAndSpaceConditions = storyboard.AndCondition(jamTimeCondition, jamAreaCondition)
        jamConditions = storyboard.AndCondition(jamTimeAndSpaceConditions, storyboard.LimitCondition(10))
        jamStory = storyboard.Story(jamConditions, [stopEffect])
        board.registerStory(jamStory)

        print("Stories loaded!")
