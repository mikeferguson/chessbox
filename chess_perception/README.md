## New perception module

Motivation:
 * pcl_ros, nodelets are unreliably maintained
 * perception could be much better with more info

Changes:
Is now a single node, finds board and pieces. Sub-modules include:
   * table_finder -- locates convex hull of table
   * piece_finder -- locates candidate pieces by clustering points above the hull, merging undersized clusters, and determining the color/weight of clusters
   * board_finder -- locates candidate board points (defined as the center of a square)
   * chess_perception -- merges all info and optimizes. publishes tf, Piece message.

Future Improvements
 * take into account expected board state (from player node)
 * piece_finder should ignore any cluster that is physically too large to be a piece
