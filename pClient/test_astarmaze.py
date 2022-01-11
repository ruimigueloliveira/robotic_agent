import pathfinder

X = 1

maze = [[X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X], #0
        [X,0,0,0,0,0,0,0,X,0,0,0,0,0,0,0,0,0,X,0,0,0,0,0,0,0,0,0,X], #1
        [X,X,X,X,X,0,X,X,X,0,X,X,X,X,X,X,X,0,X,0,X,X,X,X,X,X,X,0,X], #2
        [X,0,0,0,0,0,0,0,0,0,X,0,0,0,0,0,X,0,X,0,0,0,0,0,0,0,0,0,X], #3
        [X,0,X,X,X,X,X,X,X,0,X,0,X,X,X,X,X,0,X,X,X,0,X,X,X,X,X,0,X], #4
        [X,0,0,0,X,0,0,0,X,0,0,0,X,0,0,0,X,0,0,0,X,0,0,0,X,0,0,0,X], #5
        [X,X,X,0,X,0,X,X,X,0,X,X,X,0,X,0,X,X,X,0,X,X,X,0,X,X,X,0,X], #6
        [X,X,X,0,X,0,0,0,0,0,X,0,0,0,X,0,X,0,X,0,0,0,0,0,X,0,0,0,X], #7
        [X,X,X,0,X,X,X,X,X,X,X,0,X,X,X,0,X,0,X,X,X,0,X,0,X,X,X,0,X], #8
        [X,0,0,0,X,0,0,0,0,0,0,0,X,0,X,0,X,0,0,0,0,0,X,0,0,0,0,0,X], #9
        [X,0,X,X,X,0,X,X,X,X,X,X,X,0,X,0,X,0,X,X,X,X,X,0,X,X,X,0,X], #10
        [X,0,X,0,X,0,X,0,0,0,0,0,0,0,X,0,0,0,X,0,0,0,0,0,0,0,0,0,X], #11
        [X,0,X,0,X,0,X,0,X,X,X,X,X,0,X,X,X,X,X,0,X,X,X,X,X,X,X,0,X], #12
        [X,0,0,0,0,0,0,0,0,0,0,0,X,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,X], #13
        [X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X]] #14

beacon0 = [3, 3]
# print(maze[3][3])

beacon1 = [7, 17]
# print(maze[7][17])

beacon2 = [11, 3]
# print(maze[11][3])

beacon3 = [5, 25]
# print(maze[5][25])


path0_1 = pathfinder.search(maze, 1, beacon0, beacon1)
print("path0_1: ", path0_1)

path1_3 = pathfinder.search(maze, 1, beacon1, beacon3)
print("\npath1_3: ", path1_3)

path3_2 = pathfinder.search(maze, 1, beacon3, beacon2)
print("\npath3_2: ", path3_2)

path2_0 = pathfinder.search(maze, 1, beacon2, beacon0)
print("\npath2_0: ", path2_0)


final_path = path0_1 + path1_3 + path3_2 + path2_0

final_path = [v for i, v in enumerate(final_path) if i == 0 or v != final_path[i-1]]
print("\nfinal_path: ", final_path)
print("\nfinal_path len: ", len(final_path))