/* 
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
#define REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H

#include <boost/optional.hpp>
#include <memory>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>
#include <Eigen/Dense>

namespace reach
{
  namespace plugins
  {

    /**
 * @brief Base class solving IK at a given reach study location
 */
    class IKSolverBase
    {
    public:
      IKSolverBase()
      {
      }

      virtual ~IKSolverBase()
      {
      }

      /**
   * @brief initialize
   * @param config
   * @return
   */
      virtual bool initialize(XmlRpc::XmlRpcValue &config) = 0;

      /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given target pose starting from the input seed state.
   * If a solution is found, the resulting IK solution is saved, and the pose is scored according to the specified cost function plugin
   * @param seed
   * @param solution
   * @return a boost optional type indicating the success of the IK solution and containing the score of the solution
   */
      virtual boost::optional<double> solveIKFromSeed(const Eigen::Isometry3d &target,
                                                      const std::map<std::string, double> &seed,
                                                      std::vector<double> &solution) = 0;

      /**
   * @brief getJointNames
   * @return
   */
      virtual std::vector<std::string> getJointNames() const = 0;
    };
    typedef std::shared_ptr<IKSolverBase> IKSolverBasePtr;

  } // namespace plugins
} // namespace reach

#endif // REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
