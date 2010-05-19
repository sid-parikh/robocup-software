// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <framework/Module.hpp>
#include <framework/ConfigFile.hpp>

#include <QString>
#include <vector>
#include <list>
#include <map>

#include "BallModel.hpp"
#include "RobotModel.hpp"

/** World modeling system */
namespace Modeling
{
	class WorldModel : public Module
	{
		public:
			WorldModel(SystemState *state, const ConfigFile::WorldModel& cfg);
			~WorldModel();

			virtual void run();

		protected:

			// useful typedefs
			typedef std::vector<RobotModel::shared> RobotVector;
			typedef enum { SELF, OPP } TeamMode;

			SystemState *_state;

			// Add to opponents' shell IDs to get track map keys.
			static const int OppOffset = 256;

			/** Slots for players */
			RobotVector _selfPlayers, _oppPlayers;

			/** utility functions for update logic */
			void addRobotObseration(const Packet::Vision::Robot &robot, uint64_t timestamp,
					std::vector<RobotModel::shared>& players);
			void updateRobots(std::vector<RobotModel::shared>& players, uint64_t cur_time);
			void addRobotRxData(Packet::LogFrame::Robot& robot);
			void copyRobotState(const std::vector<RobotModel::shared>& players, TeamMode m);

			BallModel ballModel;

			/** allow for searching by robot ID (both self and opp) */
			RobotModel::RobotMap _robotMap;

			const ConfigFile::WorldModel& _config;
	};
}
