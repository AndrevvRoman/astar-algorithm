#pragma once
#include <SFML/Graphics.hpp>

#include <list>

#include "ArrayMap.hpp"

class PrettyFrontend
{
public:
    static constexpr int WINDOW_WIDTH = 800;
    static constexpr int WINDOW_HEIGHT = 800;
    static constexpr int CELL_STEP = 5;
    void draw(const ArrayMap &map)
    {
        using namespace sf;

        // Create the main window
        const float rectWidth = WINDOW_WIDTH / map.MAP_WIDTH;
        const float rectHeight = WINDOW_HEIGHT / map.MAP_HEIGHT;
        RenderWindow window(VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "A* visualization");
        std::list<RectangleShape> rects;

        for (size_t i = 0; i < map.MAP_HEIGHT; ++i)
        {
            for (size_t j = 0; j < map.MAP_WIDTH; ++j)
            {
                auto currentCell = static_cast<ArrayMap::CellType>(map.frontend[i][j]);
                if (currentCell != ArrayMap::CellType::EMPTY_POS)
                {
                    RectangleShape rect(Vector2f(rectWidth, rectHeight));
                    rect.setPosition(Vector2f(CELL_STEP + (j * rectWidth), CELL_STEP + (i * rectHeight)));
                    switch (currentCell)
                    {
                    case ArrayMap::CellType::WALL_POS:
                        rect.setFillColor(Color::White);
                        break;
                    case ArrayMap::CellType::PATH_POS:
                        rect.setFillColor(Color::Blue);
                        break;
                    case ArrayMap::CellType::START_POS:
                        rect.setFillColor(Color::Green);
                        break;
                    case ArrayMap::CellType::GOAL_POS:
                        rect.setFillColor(Color::Red);
                        break;
                    case ArrayMap::CellType::OPEN_PATH_POS:
                        rect.setFillColor(Color::Magenta);
                        break;
                    case ArrayMap::CellType::CLOSE_PATH_POS:
                        rect.setFillColor(Color::Cyan);
                        break;
                    default:
                        break;
                    }
                    rects.push_back(rect);
                }
            }
        }
        // Start the game loop
        while (window.isOpen())
        {
            Event event;
            while (window.pollEvent(event))
            {
                if (event.type == Event::Closed)
                    window.close();
            }
            window.clear();
            for (const auto r : rects)
            {
                window.draw(r);
            }
            window.display();
        }
    }
};
