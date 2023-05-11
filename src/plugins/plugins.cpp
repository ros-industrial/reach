#include <reach/plugins/boost_progress_console_logger.h>
#include <reach/plugins/console_logger.h>
#include <reach/plugins/multiplicative_evaluator.h>
#include <reach/plugins/no_op.h>
#include <reach/plugins/point_cloud_target_pose_generator.h>

#include <reach/plugin_utils.h>
EXPORT_LOGGER_PLUGIN(reach::BoostProgressConsoleLoggerFactory, BoostProgressConsoleLogger)
EXPORT_LOGGER_PLUGIN(reach::ConsoleLoggerFactory, ConsoleLogger)
EXPORT_EVALUATOR_PLUGIN(reach::MultiplicativeEvaluatorFactory, MultiplicativeEvaluator)
EXPORT_EVALUATOR_PLUGIN(reach::NoOpEvaluatorFactory, NoOpEvaluator)
EXPORT_IK_SOLVER_PLUGIN(reach::NoOpIKSolverFactory, NoOpIKSolver)
EXPORT_DISPLAY_PLUGIN(reach::NoOpDisplayFactory, NoOpDisplay)
EXPORT_TARGET_POSE_GENERATOR_PLUGIN(reach::PointCloudTargetPoseGeneratorFactory, PointCloudTargetPoseGenerator)
