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

        if (!m_font.loadFromFile(m_fontName))
        {
            std::cout << "Failed to load font for pretty drawing" << std::endl;
            return;
        }

        _initializeText();

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
                    Vector2f pos(CELL_STEP + (j * rectWidth), CELL_STEP + (i * rectHeight));
                    rect.setPosition(pos);
                    switch (currentCell)
                    {
                    case ArrayMap::CellType::WALL_POS:
                        rect.setFillColor(Color::White);
                        break;
                    case ArrayMap::CellType::PATH_POS:
                        rect.setFillColor(Color::Yellow);
                        break;
                    case ArrayMap::CellType::START_POS:
                        m_startText.setPosition(pos);
                        rect.setFillColor(Color::Green);
                        break;
                    case ArrayMap::CellType::GOAL_POS:
                        m_goalText.setPosition(pos);
                        rect.setFillColor(Color::Red);
                        break;
                    case ArrayMap::CellType::OPEN_PATH_POS:
                        rect.setFillColor(Color(144,144,144));
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
            window.draw(m_startText);
            window.draw(m_goalText);
            window.display();
        }
    }

private:
    void _initializeText()
    {
        m_startText.setString("Start");
        m_goalText.setString("Goal");

        m_startText.setFont(m_font);
        m_goalText.setFont(m_font);

        m_startText.setCharacterSize(15);
        m_goalText.setCharacterSize(15);

        m_startText.setFillColor(sf::Color::Black);
        m_goalText.setFillColor(sf::Color::Black);
    }
    sf::Text m_startText;
    sf::Text m_goalText;
    sf::Font m_font;
    const std::string m_fontName = "roboto.ttf";
};
