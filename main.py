from utils import *
from grid import Grid
from searching_algorithms import *


def print_algorithms():
    print("\nSelect a search algorithm:")
    print("press 1 for Breadth-First Search (BFS)")
    print("press 2 for Depth-First Search (DFS)")
    print("press 3 for Dijkstra’s Algorithm")
    print("press 4 for A* Search")
    print("press 5 for Uniform Cost Search (UCS)")
    print("press 6 for Iterative Deepening DFS (IDDFS)")
    print("press 7 for Iterative Deepening A* (IDA*)")
    print("\nPress SPACE to start the selected algorithm.\n")

if __name__ == "__main__":
    # setting up how big will be the display window
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))

    # set a caption for the window
    pygame.display.set_caption("Path Visualizing Algorithm")

    ROWS = 50  # number of rows
    COLS = 50  # number of columns
    grid = Grid(WIN, ROWS, COLS, WIDTH, HEIGHT)

    start = None
    end = None

    # flags for running the main loop
    run = True
    started = False

    selected_algorithm = None
    print_algorithms()

    while run:
        grid.draw()  # draw the grid and its spots
        for event in pygame.event.get():
            # verify what events happened
            if event.type == pygame.QUIT:
                run = False

            if started:
                # do not allow any other interaction if the algorithm has started
                continue  # ignore other events if algorithm started

            if pygame.mouse.get_pressed()[0]:  # LEFT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)

                if row >= ROWS or row < 0 or col >= COLS or col < 0:
                    continue  # ignore clicks outside the grid

                spot = grid.grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)
                spot = grid.grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_1:
                    selected_algorithm = "bfs"
                    print("Selected: Breadth-First Search")
                elif event.key == pygame.K_2:
                    selected_algorithm = "dfs"
                    print("Selected: Depth-First Search")
                elif event.key == pygame.K_3:
                    selected_algorithm = "dijkstra"
                    print("Selected: Dijkstra’s Algorithm")
                elif event.key == pygame.K_4:
                    selected_algorithm = "astar"
                    print("Selected: A* Search")
                elif event.key == pygame.K_5:
                    selected_algorithm = "ucs"
                    print("Selected: Uniform Cost Search")
                elif event.key == pygame.K_6:
                    selected_algorithm = "iddfs"
                    print("Selected: Iterative Deepening DFS")
                elif event.key == pygame.K_7:
                    selected_algorithm = "ida_star"
                    print("Selected: Iterative Deepening A*")

                if event.key == pygame.K_SPACE and not started:
                    # run the algorithm
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    # here you can call the algorithms
                    # bfs(lambda: grid.draw(), grid, start, end)
                    # dfs(lambda: grid.draw(), grid, start, end)
                    # astar(lambda: grid.draw(), grid, start, end)
                    # ... and the others?

                    started = True

                    # Run the selected algorithm
                    if selected_algorithm == "bfs":
                        bfs(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "dfs":
                        dfs(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "dijkstra":
                        dijkstra(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "astar":
                        astar(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "ucs":
                        uniform_cost_search(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "iddfs":
                        iterative_deepening_dfs(lambda: grid.draw(), grid, start, end)
                    elif selected_algorithm == "ida_star":
                        ida_star(lambda: grid.draw(), grid, start, end)

                    started = False

                if event.key == pygame.K_c:
                    print("Clearing the grid...")
                    start = None
                    end = None
                    grid.reset()
                    selected_algorithm = None
                    print_algorithms()
    pygame.quit()
