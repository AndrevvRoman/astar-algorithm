#pragma once
#include <list>
#include <deque>
#include <thread>

#include <SFML/Graphics.hpp>

#include "ArrayMap.hpp"
#include "MapSearchNode.hpp"

class SFML_Frontend
{
public:
    static constexpr int WINDOW_WIDTH = 800;
    static constexpr int WINDOW_HEIGHT = 800;
    static constexpr int CELL_STEP = 5;

    void instantDraw(const ArrayMap &map)
    {
        using namespace sf;

        if (!m_font.loadFromFile(m_fontName))
        {
            std::cout << "Text will be ignored. Try to run from the same directory as binary" << std::endl;
        }
        else
        {
            _initializeText();
        }

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
                        rect.setFillColor(Color(144, 144, 144));
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

    bool stepByStepDraw(const ArrayMap &map, std::deque<MapSearchNode> solution, std::deque<MapSearchNode> visited)
    {
        using namespace sf;

        if (!m_font.loadFromFile(m_fontName))
        {
            std::cout << "Text will be ignored. Try to run from the same directory as binary" << std::endl;
        }
        else
        {
            _initializeText();
        }

        // Create the main window
        const float rectWidth = WINDOW_WIDTH / map.MAP_WIDTH;
        const float rectHeight = WINDOW_HEIGHT / map.MAP_HEIGHT;
        RenderWindow window(VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "A* visualization");
        window.setFramerateLimit(60);
        std::list<RectangleShape> rects;

        // First drawing empty grid
        for (size_t i = 0; i < map.MAP_HEIGHT; ++i)
        {
            for (size_t j = 0; j < map.MAP_WIDTH; ++j)
            {
                auto currentCell = static_cast<ArrayMap::CellType>(map.frontend[i][j]);
                if (currentCell == ArrayMap::CellType::WALL_POS or
                    currentCell == ArrayMap::CellType::START_POS or
                    currentCell == ArrayMap::CellType::GOAL_POS)
                {
                    RectangleShape rect(Vector2f(rectWidth, rectHeight));
                    Vector2f pos(CELL_STEP + (j * rectWidth), CELL_STEP + (i * rectHeight));
                    rect.setPosition(pos);
                    switch (currentCell)
                    {
                    case ArrayMap::CellType::WALL_POS:
                        rect.setFillColor(Color::White);
                        break;
                    case ArrayMap::CellType::START_POS:
                        m_startText.setPosition(pos);
                        rect.setFillColor(Color::Green);
                        break;
                    case ArrayMap::CellType::GOAL_POS:
                        m_goalText.setPosition(pos);
                        rect.setFillColor(Color::Red);
                        break;
                    default:
                        break;
                    }
                    rects.push_back(rect);
                }
            }
        }

        size_t visitedNodeIndex = 1;
        size_t solutionNodeIndex = 1;

        bool stopLoop = false;
        bool isShutDown = false;

        while (!stopLoop && window.isOpen())
        {
            Event event;
            while (window.pollEvent(event))
            {
                if (event.type == Event::Closed)
                {
                    window.close();
                    isShutDown = true;
                }
            }

            if (visitedNodeIndex < visited.size())
            {
                RectangleShape rect(Vector2f(rectWidth, rectHeight));
                Vector2f pos(CELL_STEP + (visited[visitedNodeIndex].x * rectWidth), CELL_STEP + (visited[visitedNodeIndex].y * rectHeight));
                rect.setPosition(pos);
                rect.setFillColor(Color(144, 144, 144));
                rects.push_back(rect);
                visitedNodeIndex++;
            }
            // We dont want to fill the goal cell so we just skip the last node of the solution
            else if (solutionNodeIndex < solution.size() - 1)
            {
                RectangleShape rect(Vector2f(rectWidth, rectHeight));
                Vector2f pos(CELL_STEP + (solution[solutionNodeIndex].x * rectWidth), CELL_STEP + (solution[solutionNodeIndex].y * rectHeight));
                rect.setPosition(pos);
                rect.setFillColor(Color::Yellow);
                rects.push_back(rect);
                solutionNodeIndex++;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
                stopLoop = true;
            }

            // FIXME
            // In good solution we should't send to sleep the main thread to get step-by-step drawing.
            // I should implement sfml rendering in separate thread and use thread-safe queue (e.g. with mutex) or some kind
            // of observer pattern to notify sfml thread that we have to render something new on window
            // but it would be too much work for the job test case.
            std::this_thread::sleep_for(std::chrono::milliseconds(80));

            window.clear();
            for (const auto r : rects)
            {
                window.draw(r);
            }
            window.draw(m_startText);
            window.draw(m_goalText);
            window.display();
        }
        return isShutDown;
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
    const std::string m_fontName = "./roboto.ttf";
};
