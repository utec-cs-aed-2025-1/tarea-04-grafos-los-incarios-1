//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <queue>
#include <limits>
#include <cmath>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    BestFirstSearch
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;
        double heuristic; // Added for A* and BFS

        // Priority queue orders by greatest element, so we need to reverse the comparison for min-heap behavior
        bool operator > (const Entry& other) const {
            return (dist + heuristic) > (other.dist + other.heuristic);
        }
    };

    double euclidean_distance(Node* a, Node* b) {
        return std::sqrt(std::pow(a->coord.x - b->coord.x, 2) + std::pow(a->coord.y - b->coord.y, 2));
    }

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> dist;
        
        for (auto& pair : graph.nodes) {
            dist[pair.second] = std::numeric_limits<double>::infinity();
        }

        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

        dist[src] = 0;
        pq.push({src, 0, 0});

        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();

            if (current.dist > dist[current.node]) continue;
            if (current.node == dest) break;

            // Visualization
            if (current.node != src) {
                 // We don't have the edge directly here easily without looking up parent, 
                 // but we can just draw the node or reconstruct edge if needed for visited_edges.
                 // For simplicity in this structure, let's just mark visited edges when we traverse them.
            }

            for (Edge* edge : current.node->edges) {
                Node* neighbor = (edge->src == current.node) ? edge->dest : edge->src;
                double new_dist = dist[current.node] + edge->length;

                if (new_dist < dist[neighbor]) {
                    dist[neighbor] = new_dist;
                    parent[neighbor] = current.node;
                    pq.push({neighbor, new_dist, 0});
                    
                    // Add to visited edges for visualization
                    visited_edges.emplace_back(current.node->coord, neighbor->coord, sf::Color(100, 100, 100, 50), 1.0f);
                    render(graph);
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> g_score;

        for (auto& pair : graph.nodes) {
            g_score[pair.second] = std::numeric_limits<double>::infinity();
        }

        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

        g_score[src] = 0;
        pq.push({src, 0, euclidean_distance(src, dest)});

        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();

            if (current.dist > g_score[current.node]) continue;
            if (current.node == dest) break;

            for (Edge* edge : current.node->edges) {
                Node* neighbor = (edge->src == current.node) ? edge->dest : edge->src;
                double tentative_g_score = g_score[current.node] + edge->length;

                if (tentative_g_score < g_score[neighbor]) {
                    parent[neighbor] = current.node;
                    g_score[neighbor] = tentative_g_score;
                    double f_score = tentative_g_score + euclidean_distance(neighbor, dest);
                    pq.push({neighbor, tentative_g_score, euclidean_distance(neighbor, dest)}); // Entry uses dist + heuristic for priority

                    visited_edges.emplace_back(current.node->coord, neighbor->coord, sf::Color(100, 100, 100, 50), 1.0f);
                    render(graph);
                }
            }
        }

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, bool> visited;

        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

        visited[src] = true;
        pq.push({src, 0, euclidean_distance(src, dest)});

        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();

            if (current.node == dest) break;

            for (Edge* edge : current.node->edges) {
                Node* neighbor = (edge->src == current.node) ? edge->dest : edge->src;
                
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    parent[neighbor] = current.node;
                    pq.push({neighbor, 0, euclidean_distance(neighbor, dest)});

                    visited_edges.emplace_back(current.node->coord, neighbor->coord, sf::Color(100, 100, 100, 50), 1.0f);
                    render(graph);
                }
            }
        }

        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        // sf::sleep(sf::milliseconds(1)); // Reduced sleep for faster execution
        window_manager->clear();
        
        // We can't easily draw the whole graph every frame if it's too big, but let's try following the original pattern
        // The original code in gui.h calls graph.draw() then path_finding_manager.draw().
        // Here we are inside path_finding_manager, so we can't easily call graph.draw() unless we pass it or assume it's static/global.
        // However, the prompt says "Implement render()".
        // The `graph` object is passed to `exec`, but not stored.
        // Let's just draw the visited edges and the path so far?
        // Actually, the `gui.h` main loop handles the clearing and drawing of the graph.
        // If we want to animate *during* the algorithm, we need to trigger the window display here.
        
        // NOTE: To properly animate within the loop without blocking the main thread forever or freezing the UI,
        // we usually process events. But for this simple task, we might just display.
        // But we need to redraw the graph background too.
        // Since we don't have access to the graph object in `render` (it's not a member), 
        // we might need to pass it or just draw the visited edges on top of black?
        // The user asked for "simple possible".
        // Let's just do a simple display of what we have.
        
        // ISSUE: We don't have access to `graph` here to redraw the background nodes/edges.
        // The `exec` function receives `graph`. I should probably pass `graph` to `render` or store a reference.
        // But `render` signature in the file is `void render()`.
        // I will modify `render` to take `Graph& graph` as argument, or just not clear the screen and draw on top?
        // If I don't clear, it will look messy if I move things, but here I am just adding lines.
        // So drawing on top might work if I don't call clear().
        // BUT `window_manager->display()` swaps buffers. So I need to redraw everything.
        
        // Let's change `render` signature to take `Graph& graph`.
    }
    
    void render(Graph& graph) {
         window_manager->clear();
         graph.draw();
         draw(true); // Draw visited edges and current path (which is empty so far)
         window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        Node* current = dest;
        path.clear();
        
        if (parent.find(dest) == parent.end()) {
            return; // No path found
        }

        while (current != src) {
            Node* prev = parent[current];
            path.emplace_back(prev->coord, current->coord, sf::Color::Blue, 2.0f);
            current = prev;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        reset(); // Clear previous path

        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case BestFirstSearch:
                best_first_search(graph);
                break;
            default:
                break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
