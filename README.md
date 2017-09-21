# Brain-network-visualization
Utilities for visualizing brain and connection (for off-shelf figure)

###  Brain_surface_connection.py  ###
The following main features
1. Rendering brain surface and connetions
2. Show and save as off-shelf figure or animation movie
3. Fast rendering since Python/Mayavi call OpenGL

Surface format:
```
3                                   # node number
-2.886495 24.886947 15.909558       # node coord
-13.916695 -58.985245 19.655311
-40.415528 8.968353 8.515955
-47.707717 -48.107485 17.630739
-26.480126 44.708967 32.840302
4                                   # triangle number
1 2 3                               # triangle edge
1 3 5
2 3 5
3 4 5
```

Network format:
```
vertex  3                            # node number
-2.886495 24.886947 15.909558        # node coord
-13.916695 -58.985245 19.655311
-40.415528 8.968353 8.515955
edge 4                               # conneciton number
1 2                                  # connection
1 3 
```

