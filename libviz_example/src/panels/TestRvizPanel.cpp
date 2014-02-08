/*
 * TestRvizPanel.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: ros-industrial
 */

#include <libviz_example/panels/TestRvizPanel.h>
#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>

namespace libviz_example{namespace panels{


TestRvizPanel::TestRvizPanel(QWidget* parent)
:rviz::Panel(parent),
 text_entered_("null")
{
	// TODO Auto-generated constructor stub
	//init();
	ROS_INFO_STREAM("Constructor invoked");
}

TestRvizPanel::~TestRvizPanel()
{
	std::cout<<"\nTestRvizPanel Destructor called"<<std::endl;
}

void TestRvizPanel::load(const rviz::Config& config)
{
	rviz::Panel::load(config);
	QString text_entry;
	ROS_INFO_STREAM("Reading config file");
	if(config.mapGetString("TextEntry",&text_entry))
	{
		text_entered_ = text_entry.toStdString();
	}
	ROS_INFO_STREAM("Finish reading config file");
}

void TestRvizPanel::save(rviz::Config config) const
{
	ROS_INFO_STREAM("Saving configuration");
	rviz::Panel::save(config);
	config.mapSetValue("TextEntry",QString::fromStdString( text_entered_));
}

void TestRvizPanel::onInitialize()
{

	// creating top button-text entry layout
	ROS_INFO_STREAM("Creating submenu 1 elements");
//	QHBoxLayout* sub_layout1  = new QHBoxLayout(this);
//	sub_layout1->addWidget(new QLabel("Edit Test"));
//	line_edit_ = new QLineEdit();
//	line_edit_->insert(QString::fromStdString(text_entered_));
//	button_ = new QPushButton(QString("confirm"));
//	sub_layout1->addWidget(line_edit_);
//	sub_layout1->addWidget(button_);


	// creating spinner elements
	ROS_INFO_STREAM("Creating submenu 2 elements");
//	QHBoxLayout* sub_layout2 = new QHBoxLayout(this);
//	sub_layout2->addWidget(new QLabel("spinner"));
//	spin_box_ = new QDoubleSpinBox();
//	spin_box_->setMinimum(0.0f);
//	spin_box_->setMaximum(1.0f);
//	spin_box_->setSingleStep(0.1f);
//	sub_layout2->addWidget(spin_box_);


	// creating slider elements
	ROS_INFO_STREAM("Creating submenu 3 elements");
	test_widget_ = new QWidget(this);
	QHBoxLayout* sub_layout3 = new QHBoxLayout(this);
	sub_layout3->addWidget(new QLabel("Slider"));
	slider_ = new QSlider(Qt::Horizontal);
	slider_->setMinimum(1);
	slider_->setMaximum(100);
	//slider_->setTickInterval(10);
	//slider_->setSingleStep(1);
	sub_layout3->addWidget(slider_);
	test_widget_->setLayout(sub_layout3);

	// creating main layout
	QVBoxLayout* main_layout = new QVBoxLayout(this);
	main_layout->addWidget(new QLabel("TestRvizPanel"));
	//main_layout->addLayout(sub_layout1);
	main_layout->addWidget(test_widget_);

	ROS_INFO_STREAM("Adding elements to main layout");
	//setLayout(main_layout);

	ROS_INFO_STREAM("Connecting signals to slots");
//	connect(line_edit_,SIGNAL(textEdited(const QString&)),
//			this,SLOT(text_edited_handler(const QString& )));
//	connect(button_,SIGNAL(clicked()),this,SLOT(button_handler()));
//	connect(spin_box_,SIGNAL(valueChanged(double)),this,SLOT(spinner_handler(double)));
//	connect(slider_,SIGNAL(valueChanged(int)),this,SLOT(slider_handler(int)));

	ROS_INFO_STREAM("Connected signals to slots. Finished on initialized");
}

void TestRvizPanel::slider_handler(int val)
{
	ROS_INFO_STREAM("Slider Value: "<<val);
}

void TestRvizPanel::button_handler()
{
	ROS_INFO_STREAM("Button clicked, line edit content: "<<text_entered_);
}

void TestRvizPanel::spinner_handler(double val)
{
	ROS_INFO_STREAM("Spinner value: "<<val);
}

void TestRvizPanel::text_edited_handler(const QString &text)
{
	ROS_INFO_STREAM("text edited handler");
	text_entered_ = text.toStdString();
}

}}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(libviz_example::panels::TestRvizPanel,rviz::Panel )
