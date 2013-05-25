## New perception module

Motivation:
 * pcl_ros, nodelets are unreliably maintained
 * perception could be much better with more info
 * eliminate need for camera_turnpike

Changes:
Is now a single node, finds board and pieces. Sub-modules include:
   * board_finder -- finds a transform between the chess board and world. Publishes tf.
   * piece_finder -- finds pieces above the board.
   * chess_perception -- merges all info (and optimizes?). publishes board/piece message.

Future Improvements
 * make board_finder robust to missing/added intersections
 * improve board_finder intersection acceptance test
 * take into account expected board state (from player node)
 * piece_finder should ignore any cluster that is physically too large to be a piece
