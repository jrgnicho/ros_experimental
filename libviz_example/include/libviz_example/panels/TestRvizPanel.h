/*
 * TestRvizPanel.h
 *
 *  Created on: Feb 3, 2014
 *      Author: ros-industrial
 */

#ifndef TESTRVIZPANEL_H_
#define TESTRVIZPANEL_H_

# include <ros/ros.h>
# include <rviz/panel.h>

class QWidget;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;
class QSlider;

namespace libviz_example{ namespace panels{

class TestRvizPanel: public rviz::Panel
{
Q_OBJECT // QT macro
public:
	TestRvizPanel(QWidget* parent = 0);
	virtual ~TestRvizPanel();

	virtual void onInitialize();

protected Q_SLOTS: // qt event callbacks

	void slider_handler(int val);
	void button_handler();
	void spinner_handler(double val);
	void text_edited_handler(const QString &text);

	// rviz::Panel virtual functions
	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;


protected:

	QWidget* test_widget_;
	QLineEdit* line_edit_;
	QPushButton* button_;
	QDoubleSpinBox* spin_box_;
	QSlider* slider_;
	//ros::NodeHandle nh_;
	std::string text_entered_;

};

}}

#endif /* TESTRVIZPANEL_H_ */
