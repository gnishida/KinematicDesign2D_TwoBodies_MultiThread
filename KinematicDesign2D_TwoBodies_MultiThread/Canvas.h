#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <kinematics.h>
#include <QTimer>
#include "Shape.h"
#include "Operation.h"
#include "History.h"
#include "Design.h"

class MainWindow;

namespace canvas {

	class Canvas : public QWidget {
		Q_OBJECT

	public:
		static enum { MODE_SELECT = 0, MODE_MOVE, MODE_ROTATION, MODE_RESIZE, MODE_FIXED_RECTANGLE, MODE_FIXED_CIRCLE, MODE_FIXED_POLYGON, MODE_MOVING_RECTANGLE, MODE_MOVING_CIRCLE, MODE_MOVING_POLYGON, MODE_LINKAGE_REGION, MODE_LINKAGE_AVOIDANCE, MODE_KINEMATICS };
		static enum { LINKAGE_WATTI = 0 };

	public:
		MainWindow* mainWin;
		bool ctrlPressed;
		bool shiftPressed;

		int mode;
		boost::shared_ptr<Operation> operation;
		boost::shared_ptr<canvas::Shape> current_shape;
		boost::shared_ptr<canvas::Shape> selected_shape;
		std::vector<boost::shared_ptr<canvas::Shape>> copied_shapes;
		canvas::Design design;
		History history;
		
		std::vector<boost::shared_ptr<kinematics::LinkageSynthesis>> synthesis;

		std::vector<kinematics::Kinematics> kinematics;
		kinematics::Solution selected_solution; // currently selected solution
		std::vector<kinematics::Solution> initial_solutions;
		std::vector<kinematics::Solution> solutions;
		std::pair<int, int> selectedJoint;
		std::vector<std::vector<glm::dmat3x3>> poses;
		std::vector<kinematics::Object25D> fixed_bodies;
		std::vector<kinematics::Object25D> merged_fixed_bodies;
		std::vector<kinematics::Object25D> moving_bodies;
		std::vector<std::vector<glm::dvec2>> linkage_region_pts;
		std::vector<std::vector<glm::dvec2>> linkage_avoidance_pts;
		int linkage_type;
		QTimer* animation_timer;
		bool collision_check;
		bool restrict_motion_range;
		bool show_solutions;
		bool show_grid_lines;
		bool show_input_poses;
		bool show_linkage;

		// parameters for sampling and particle filter
		int num_samples;
		double stddev_position;
		double stddev_orientation;
		bool avoid_branch_defect;
		double min_transmission_angle;
		std::vector<double> weights;
		int num_particles;
		int num_pf_iterations;
		bool record_pf;

		QPointF prev_mouse_pt;
		QPointF origin;
		double scale;

	public:
		Canvas(MainWindow* mainWin);
		~Canvas();

		void clear();
		void selectAll();
		void unselectAll();
		void deleteSelectedShapes();
		void undo();
		void redo();
		void copySelectedShapes();
		void pasteCopiedShapes();
		void setMode(int mode);
		void addLayer();
		void insertLayer();
		void deleteLayer();
		void setLayer(int layer_id);
		void open(const QString& filename);
		void save(const QString& filename);
		void run();
		void runBackward();
		void stop();
		void speedUp();
		void speedDown();
		void invertSpeed();
		void stepForward();
		void stepBackward();
		glm::dvec2 screenToWorldCoordinates(const glm::dvec2& p);
		glm::dvec2 screenToWorldCoordinates(double x, double y);
		glm::dvec2 worldToScreenCoordinates(const glm::dvec2& p);

		void calculateSolutions(int linkage_type);
		void updateSolutions(int linkage_type);
		void constructKinematics();
		int findSolution(const std::vector<kinematics::Solution>& solutions, const glm::dvec2& pt, int joint_id);

	public slots:
		void animation_update();

	protected:
		void paintEvent(QPaintEvent* e);
		void mousePressEvent(QMouseEvent* e);
		void mouseMoveEvent(QMouseEvent* e);
		void mouseReleaseEvent(QMouseEvent* e);
		void mouseDoubleClickEvent(QMouseEvent* e);
		void wheelEvent(QWheelEvent* e);
		void resizeEvent(QResizeEvent *e);

	public:
		void keyPressEvent(QKeyEvent* e);
		void keyReleaseEvent(QKeyEvent* e);
	};

}

#endif // CANVAS_H
