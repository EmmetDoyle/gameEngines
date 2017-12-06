#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Assignment :
		public Game
	{
	private:

	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();


		//body
		float bodyWidth = 20;
		float bodyLength = 10;
		float bodyDepth = 10;

		//arms
		float armWidth = 10;
		float armLength = 2;
		float armDepth = 2;

		shared_ptr<PhysicsController> CreateCrawler(glm::vec3 position);

	};
}
