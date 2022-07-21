#pragma once
#include <SFML/Graphics.hpp>

#include <list>

#include "ArrayMap.hpp"

class PrettyFrontend
{
public:
    void draw(const ArrayMap &map)
    {
        // Create the main window
        const float rectWidth = 800 / map.MAP_WIDTH;
        const float rectHeight = 800 / map.MAP_HEIGHT;
        sf::RenderWindow window(sf::VideoMode(800, 800), "A* visualisation");
        std::list<sf::RectangleShape> rects;

        for (size_t i = 0; i < map.MAP_HEIGHT; ++i)
        {
            for (size_t j = 0; j < map.MAP_WIDTH; ++j)
            {
                auto currentCell = static_cast<ArrayMap::CellType>(map.frontend[i][j]);
                if (currentCell != ArrayMap::CellType::EMPTY_POS)
                {
                    sf::RectangleShape rect(sf::Vector2f(rectWidth, rectHeight));
                    rect.setPosition(sf::Vector2f(5 + (j * rectWidth), 5 + (i * rectHeight)));
                    switch (currentCell)
                    {
                    case ArrayMap::CellType::WALL_POS:
                        rect.setFillColor(sf::Color::White);
                        break;
                    case ArrayMap::CellType::PATH_POS:
                        rect.setFillColor(sf::Color::Blue);
                        break;
                    case ArrayMap::CellType::START_POS:
                        rect.setFillColor(sf::Color::Green);
                        break;
                    case ArrayMap::CellType::GOAL_POS:
                        rect.setFillColor(sf::Color::Red);
                        break;
                    case ArrayMap::CellType::OPEN_PATH_POS:
                        rect.setFillColor(sf::Color::Magenta);
                        break;
                    case ArrayMap::CellType::CLOSE_PATH_POS:
                        rect.setFillColor(sf::Color::Cyan);
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
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
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
