#include <iostream>
#include <string>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <vector>
#include <list>
#include <time.h>

using namespace std;

class AStarPathFinding {

private:

	struct Node {
		bool isObstacle = false;
		bool isVisited = false;
		float fGlobalGoal;
		float fLocalGoal;
		int x;
		int y;
		vector<Node*> neighbours;
		Node* parent;
	};

	Node* nodes = nullptr;

	int nMapWidth = 50;
	int nMapHeight = 50;

	Node* nodeStart = nullptr;
	Node* nodeEnd = nullptr;

	bool initNodes() {

		nodes = new Node[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].x = x;
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].isObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].isVisited = false;
			}

		for (int y = 0; y < nMapHeight; y++)
		{
			for (int x = 0; x < nMapWidth; x++)
			{
				if (y > 0)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y - 1) * nMapHeight + (x + 0)]);
				if (y < nMapHeight - 1)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y + 1) * nMapHeight + (x + 0)]);
				if (x > 0)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y + 0) * nMapHeight + (x - 1)]);
				if (x < nMapWidth - 1)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y + 0) * nMapHeight + (x + 1)]);


				if (y > 0 && x > 0)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y - 1) * nMapHeight + (x - 1)]);
				if (y < nMapHeight - 1 && x>0)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y + 1) * nMapHeight + (x - 1)]);
				if (y > 0 && x < nMapWidth - 1)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y - 1) * nMapHeight + (x + 1)]);
				if (y < nMapHeight - 1 && x < nMapWidth - 1)
					nodes[y * nMapHeight + x].neighbours.push_back(&nodes[(y + 1) * nMapHeight + (x + 1)]);


			}
		}

		nodeStart = &nodes[0];
		nodeEnd = &nodes[(nMapWidth * nMapHeight) - 1];
		return true;
	}

	bool resetNodes() {
		for (int y = 0; y < nMapHeight; y++) {
			for (int x = 0; x < nMapWidth; x++)
			{
				nodes[y * nMapHeight + x].isVisited = false;
				nodes[y * nMapHeight + x].fGlobalGoal = INFINITY;
				nodes[y * nMapHeight + x].fLocalGoal = INFINITY;
				nodes[y * nMapHeight + x].parent = nullptr;
			}
		}
		return true;
	}


	bool solveAStar() {

		resetNodes();

		auto distance = [](Node* a, Node* b) {
			return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
		};

		auto heuristic = [distance](Node* a, Node* b) {
			return distance(a, b);
		};

		Node* currentNode = nodeStart;
		nodeStart->fLocalGoal = 0.0f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);


		vector<Node*> vectNotTestedNode;
		vectNotTestedNode.push_back(nodeStart);

		while (!vectNotTestedNode.empty() && currentNode != nodeEnd) {

			sort(vectNotTestedNode.begin(), vectNotTestedNode.end(), [](Node* lhs, Node* rhs) {return (lhs->fGlobalGoal < rhs->fGlobalGoal); });
			while (!vectNotTestedNode.empty() && vectNotTestedNode.front()->isVisited) {
				vectNotTestedNode.erase(vectNotTestedNode.begin());

			}

			if (vectNotTestedNode.empty())
				break;

			currentNode = vectNotTestedNode.front();
			currentNode->isVisited = true;

			for (auto nodeNeighbour : currentNode->neighbours) {
				if (!nodeNeighbour->isVisited && !nodeNeighbour->isObstacle)
					vectNotTestedNode.push_back(nodeNeighbour);

				float fPossiblyLowerGoal = currentNode->fLocalGoal + distance(currentNode, nodeNeighbour);

				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal) {
					nodeNeighbour->parent = currentNode;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);

				}
			}
		}
		return true;
	}

public:

	AStarPathFinding() {
		bool initNodes();
	}

	bool renderGrid() {



		initNodes();

		int W = 800;
		int H = 800;
		sf::RenderWindow window(sf::VideoMode(W, H), "A* Shortest Path Algorithm", sf::Style::Close | sf::Style::Titlebar);
		window.setKeyRepeatEnabled(false);

		float colNodeSize = window.getSize().x / nMapWidth;
		float rowNodeSize = window.getSize().y / nMapHeight;


		sf::RectangleShape shape(sf::Vector2f(colNodeSize - (colNodeSize * .2), colNodeSize - (colNodeSize * .2)));
		sf::VertexArray line(sf::TriangleStrip, 4);

		sf::Color DarkBlue = sf::Color::Color(sf::Uint8(0), sf::Uint8(0), sf::Uint8(150), sf::Uint8(255));

		while (window.isOpen())
		{


			sf::Vector2i localMousePosition(sf::Mouse::getPosition(window));
			int selectedNodeX = localMousePosition.x / colNodeSize;
			int selectedNodeY = localMousePosition.y / rowNodeSize;

			sf::Event event;

			while (window.pollEvent(event))
			{
				if (event.type == sf::Event::Closed)
					window.close();
			}

			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {

				if (selectedNodeX >= 0 && selectedNodeX < nMapWidth)
					if (selectedNodeY >= 0 && selectedNodeY < nMapHeight) {
						if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
							nodeStart = &nodes[selectedNodeY * nMapWidth + selectedNodeX];
						else if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))
							nodeEnd = &nodes[selectedNodeY * nMapWidth + selectedNodeX];
						else if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
							for (auto n : nodes[selectedNodeY * nMapWidth + selectedNodeX].neighbours) {
								n->isObstacle = true;
							}
							nodes[selectedNodeY * nMapWidth + selectedNodeX].isObstacle = true;
						}
						else
							nodes[selectedNodeY * nMapWidth + selectedNodeX].isObstacle = true;
						solveAStar();
					}

			}
			if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
				if (sf::Keyboard::isKeyPressed(sf::Keyboard::F)) {
					for (auto n : nodes[selectedNodeY * nMapWidth + selectedNodeX].neighbours) {
						n->isObstacle = false;
					}
					nodes[selectedNodeY * nMapWidth + selectedNodeX].isObstacle = false;
				}
				else {
					nodes[selectedNodeY * nMapWidth + selectedNodeX].isObstacle = false;

				}
				solveAStar();
			}


			window.clear();



			for (int y = 0; y < nMapHeight; y++)
			{
				for (int x = 0; x < nMapWidth; x++)
				{
					for (auto n : nodes[y * nMapHeight + x].neighbours)
					{

						line[0].color = DarkBlue;
						line[1].color = DarkBlue;
						line[2].color = DarkBlue;
						line[3].color = DarkBlue;


						if (n->x - x != 0) {
							line[0].position = sf::Vector2f(x * colNodeSize + colNodeSize / 2, y * rowNodeSize + rowNodeSize / 2 - (rowNodeSize * .05));
							line[1].position = sf::Vector2f(x * colNodeSize + colNodeSize / 2, y * rowNodeSize + rowNodeSize / 2 + (rowNodeSize * .05));
							line[2].position = sf::Vector2f(n->x * colNodeSize + colNodeSize / 2, n->y * rowNodeSize + rowNodeSize / 2 - (rowNodeSize * .05));
							line[3].position = sf::Vector2f(n->x * colNodeSize + colNodeSize / 2, n->y * rowNodeSize + rowNodeSize / 2 + (rowNodeSize * .05));
						}
						else {
							line[0].position = sf::Vector2f(x * colNodeSize + colNodeSize / 2 - (colNodeSize * .05), y * rowNodeSize + rowNodeSize / 2);
							line[1].position = sf::Vector2f(x * colNodeSize + colNodeSize / 2 + (colNodeSize * .05), y * rowNodeSize + rowNodeSize / 2);
							line[2].position = sf::Vector2f(n->x * colNodeSize + colNodeSize / 2 - (colNodeSize * .05), n->y * rowNodeSize + rowNodeSize / 2);
							line[3].position = sf::Vector2f(n->x * colNodeSize + colNodeSize / 2 + (colNodeSize * .05), n->y * rowNodeSize + rowNodeSize / 2);
						}
						window.draw(line);
					}
				}
			}

			for (int y = 0; y < nMapHeight; y++)
			{
				for (int x = 0; x < nMapWidth; x++)
				{
					shape.setPosition(sf::Vector2f(x * colNodeSize, y * rowNodeSize));

					if (nodes[y * nMapHeight + x].isObstacle) {
						shape.setFillColor(sf::Color::White);
					}
					else if (nodes[y * nMapHeight + x].isVisited) {
						shape.setFillColor(sf::Color::Blue);
					}
					else {
						shape.setFillColor(DarkBlue);
					}


					if (&nodes[y * nMapHeight + x] == nodeStart) {
						shape.setFillColor(sf::Color::Green);
					}
					else if (&nodes[y * nMapHeight + x] == nodeEnd) {
						shape.setFillColor(sf::Color::Red);
					}
					else if (nodes[y * nMapHeight + x].isVisited) {
						shape.setFillColor(sf::Color::Blue);
					}
					window.draw(shape);
				}
			}

			if (nodeEnd != nullptr) {

				Node* p = nodeEnd;
				while (p->parent != nullptr) {
					line[0].color = sf::Color::Yellow;
					line[1].color = sf::Color::Yellow;
					line[2].color = sf::Color::Yellow;
					line[3].color = sf::Color::Yellow;
					if (p->x - p->parent->x != 0) {
						line[0].position = sf::Vector2f(p->x * colNodeSize + colNodeSize / 2, p->y * rowNodeSize + rowNodeSize / 2 - (rowNodeSize * .05));
						line[1].position = sf::Vector2f(p->x * colNodeSize + colNodeSize / 2, p->y * rowNodeSize + rowNodeSize / 2 + (rowNodeSize * .05));
						line[2].position = sf::Vector2f(p->parent->x * colNodeSize + colNodeSize / 2, p->parent->y * rowNodeSize + rowNodeSize / 2 - (rowNodeSize * .05));
						line[3].position = sf::Vector2f(p->parent->x * colNodeSize + colNodeSize / 2, p->parent->y * rowNodeSize + rowNodeSize / 2 + (rowNodeSize * .05));
					}
					else {
						line[0].position = sf::Vector2f(p->x * colNodeSize + colNodeSize / 2 - (colNodeSize * .05), p->y * rowNodeSize + rowNodeSize / 2);
						line[1].position = sf::Vector2f(p->x * colNodeSize + colNodeSize / 2 + (colNodeSize * .05), p->y * rowNodeSize + rowNodeSize / 2);
						line[2].position = sf::Vector2f(p->parent->x * colNodeSize + colNodeSize / 2 - (colNodeSize * .05), p->parent->y * rowNodeSize + rowNodeSize / 2);
						line[3].position = sf::Vector2f(p->parent->x * colNodeSize + colNodeSize / 2 + (colNodeSize * .05), p->parent->y * rowNodeSize + rowNodeSize / 2);
					}
					window.draw(line);
					p = p->parent;
				}
			}
			window.display();

		}
		return true;
	}

};

int main()
{
	AStarPathFinding aStar;
	aStar.renderGrid();

	return 0;
}