package s3.ai;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import s3.base.S3;
import s3.entities.S3PhysicalEntity;
import s3.util.Pair;


public class AStar {

	// Search parameters, stored as integer tile coordinates.
	int start_x, start_y;
	int goal_x, goal_y;
	S3PhysicalEntity entity;
	S3 game;

	// Map dimensions (in tiles).
	int mapWidth, mapHeight;

	/**
	 * Internal node used by the A* search.
	 */
	static class Node {
		int x, y;
		double g;   // cost from start to this node
		double f;   // g + heuristic
		Node parent;

		Node(int x, int y, double g, double f, Node parent) {
			this.x = x;
			this.y = y;
			this.g = g;
			this.f = f;
			this.parent = parent;
		}
	}

	public static int pathDistance(double start_x, double start_y, double goal_x, double goal_y,
			S3PhysicalEntity i_entity, S3 the_game) {
		AStar a = new AStar(start_x,start_y,goal_x,goal_y,i_entity,the_game);
		List<Pair<Double, Double>> path = a.computePath();
		if (path!=null) return path.size();
		return -1;
	}

	public AStar(double start_x, double start_y, double goal_x, double goal_y,
			S3PhysicalEntity i_entity, S3 the_game) {
		// Coordinates can be treated as integers (tile positions).
		this.start_x = (int) start_x;
		this.start_y = (int) start_y;
		this.goal_x = (int) goal_x;
		this.goal_y = (int) goal_y;
		this.entity = i_entity;
		this.game = the_game;
		this.mapWidth = the_game.getMap().getWidth();
		this.mapHeight = the_game.getMap().getHeight();
	}

	public List<Pair<Double, Double>> computePath() {
		// Trivial case: already at the goal. Path is empty (start is excluded).
		if (start_x == goal_x && start_y == goal_y) {
			return new ArrayList<Pair<Double, Double>>();
		}

		// If the goal cell itself is blocked there is no valid path.
		if (!isFree(goal_x, goal_y)) {
			return null;
		}

		// 4-connected movement (up/down/left/right). Change to include
		// diagonals if desired.
		final int[] dx = { 1, -1, 0, 0 };
		final int[] dy = { 0, 0, 1, -1 };

		PriorityQueue<Node> open = new PriorityQueue<Node>(new java.util.Comparator<Node>() {
			public int compare(Node a, Node b) {
				return Double.compare(a.f, b.f);
			}
		});

		// Best known g-cost for each visited cell, keyed by a flat index.
		HashMap<Integer, Double> bestG = new HashMap<Integer, Double>();

		Node startNode = new Node(start_x, start_y, 0.0,
				heuristic(start_x, start_y), null);
		open.add(startNode);
		bestG.put(key(start_x, start_y), 0.0);

		while (!open.isEmpty()) {
			Node current = open.poll();

			// Skip stale entries (a better path to this cell was found later).
			Double recordedG = bestG.get(key(current.x, current.y));
			if (recordedG != null && current.g > recordedG) {
				continue;
			}

			// Goal reached: reconstruct the path.
			if (current.x == goal_x && current.y == goal_y) {
				return reconstructPath(current);
			}

			for (int i = 0; i < dx.length; i++) {
				int nx = current.x + dx[i];
				int ny = current.y + dy[i];

				if (nx < 0 || ny < 0 || nx >= mapWidth || ny >= mapHeight) {
					continue;
				}

				// The goal cell is always allowed as a destination even if it
				// currently reports a collision with the moving entity itself;
				// intermediate cells must be free.
				if (!(nx == goal_x && ny == goal_y) && !isFree(nx, ny)) {
					continue;
				}

				double tentativeG = current.g + 1.0;
				int nkey = key(nx, ny);
				Double knownG = bestG.get(nkey);

				if (knownG == null || tentativeG < knownG) {
					bestG.put(nkey, tentativeG);
					double f = tentativeG + heuristic(nx, ny);
					open.add(new Node(nx, ny, tentativeG, f, current));
				}
			}
		}

		// Open list exhausted without reaching the goal: no path exists.
		return null;
	}

	/**
	 * Manhattan distance heuristic (admissible for 4-connected grids with
	 * unit step costs).
	 */
	private double heuristic(int x, int y) {
		return Math.abs(x - goal_x) + Math.abs(y - goal_y);
	}

	/**
	 * Reconstructs the path from the goal node back to the start, returning it
	 * in start->goal order. The start position is excluded; the goal is
	 * included, matching what WTroop expects.
	 */
	private List<Pair<Double, Double>> reconstructPath(Node goalNode) {
		List<Pair<Double, Double>> path = new ArrayList<Pair<Double, Double>>();
		Node n = goalNode;
		while (n != null && n.parent != null) {
			path.add(new Pair<Double, Double>((double) n.x, (double) n.y));
			n = n.parent;
		}
		Collections.reverse(path);
		return path;
	}

	/**
	 * Checks whether the entity can occupy cell (x,y) without colliding with
	 * the level or other units. This is done by temporarily placing the entity
	 * at (x,y), querying the game, and then restoring the original position.
	 */
	private boolean isFree(int x, int y) {
		int oldX = entity.getX();
		int oldY = entity.getY();

		entity.setX(x);
		entity.setY(y);
		boolean free = (game.anyLevelCollision(entity) == null);

		entity.setX(oldX);
		entity.setY(oldY);
		return free;
	}

	private int key(int x, int y) {
		return y * mapWidth + x;
	}

}
