/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/rqt_platform_panda_joint_position.h"
#include <pluginlib/class_list_macros.h>

#include <controller_manager_msgs/ListControllers.h>
// cf /opt/ros/indigo/include/controller_manager_msgs

// Service GetJointVelocity
//#include <kuka_lwr_controllers/GetJointVelocity.h>

#include <math.h>
/*#include <QtCore/QTextStream>
#include <QtCore/QMetaType>
#include <QtGui/QHeaderView>*/

#include <QTextStream>
#include <QMetaType>
#include <QHeaderView>

namespace platform_panda_plugins_ns {
	
	JointPositionPlugin::JointPositionPlugin()
	: rqt_gui_cpp::Plugin(), widget_global_(0), widget_positions_(0), tab_widget_(0), vlayout_global_(0), vlayout_positions_(0), button_send_positions_(0), firstTime_(0), plot_checked_(0), position_sliders_(0)
	{
		setObjectName("Plugin Joint Position");
		qRegisterMetaType<QVector<double> >("QVector<double>");
	}
	
	void JointPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		// create a main widget
		widget_global_ = new QWidget();
		widget_global_->setWindowTitle("Main widget");
		
		// create a main widget for sliders position
		widget_positions_ = new QWidget();
		widget_positions_->setWindowTitle("Joint Position");
		
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->setObjectName("vertical_layout_global");
		
		vlayout_positions_ = new QVBoxLayout();
		vlayout_positions_->setObjectName("vertical_layout_positions");
		
		 // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("panda_1");
        //ns_combo_->addItem("panda_2");
		
		connect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
		
		vlayout_global_->addWidget(ns_combo_);
		
		position_sliders_ = new QtPositionSliders();
		
		vlayout_positions_->addWidget(position_sliders_);
		
		/* End : Plot and Curve specifications */
		
		button_send_positions_ = new QPushButton("Send Position");
		button_send_positions_->setToolTip("Send sliders positions to ROS position controller");
		connect(button_send_positions_, SIGNAL(pressed()), this, SLOT(sendPosition()));
		
		button_go_home_ = new QPushButton("GO HOME");
		button_go_home_->setToolTip("Send Home Position to ROS position controller");
		connect(button_go_home_, SIGNAL(pressed()), this, SLOT(sendGoHome()));
		
		
		vlayout_positions_->addWidget(button_send_positions_);
		vlayout_positions_->addWidget(button_go_home_);
		
		// set widget_positions_  layout
		widget_positions_->setLayout(vlayout_positions_);
		
		tab_widget_ = new QTabWidget();
		tab_widget_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
		
		tab_widget_->addTab(widget_positions_,"Sliders Position");
		
		vlayout_global_->addWidget(tab_widget_);
		vlayout_global_->setStretchFactor(tab_widget_, 1);
		
		plot_checked_ = new platform_panda_plugins_ns::QtPlotChecked(widget_global_, QString("Movement of the Panda Joints"), QString("Joint Value (radian)"), QString("Time (sec)"), QPair<double,double>((-3.0718), (3.7525)));
		
		vlayout_global_->addWidget(plot_checked_);
		
		// set widget_global_  layout
		widget_global_->setLayout(vlayout_global_);
		
		context.addWidget(widget_global_);
		
		QVector<double> vect_init_joint_values;
		vect_init_joint_values.resize(7);
		vect_init_joint_values.fill(0);
		
		map_selected_joint_values_.insert("panda_1",vect_init_joint_values);
		//map_selected_joint_values_.insert("panda_2",vect_init_joint_values);
		
		map_current_joint_state_values_.insert("panda_1",vect_init_joint_values);
		//map_current_joint_state_values_.insert("panda_2",vect_init_joint_values);
		
		map_sliders_is_init_.insert("panda_1",false);
		//map_sliders_is_init_.insert("panda_2",false);
		
		setupROSComponents_();
		
		timer_ = new QTimer(this);

		// setup signal and slot
		connect(timer_, SIGNAL(timeout()), this, SLOT(doUpdateCurves()));

		// msec
		timer_->start(100);
	}
	
	void JointPositionPlugin::ns_combo_changed(int index)
	{
		resetSlidersPositions();
	}
	
	void JointPositionPlugin::sendGoHome()
	{
	    QVector<double> vect_home_joint_values;
        QMap<QString, QVector<double> > map_go_home_joint_values;
        
		vect_home_joint_values.resize(7);
		// [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785
		vect_home_joint_values[0] = 0.0;
		vect_home_joint_values[1] = 0.0;
		vect_home_joint_values[1] = 0.0;
		vect_home_joint_values[3] = -1.57;
		vect_home_joint_values[4] = 0.0;
		vect_home_joint_values[5] = 1.57;
		vect_home_joint_values[6] = 0.785;
		
		map_go_home_joint_values.insert(ns_combo_->currentText(),vect_home_joint_values);
	
	    joint_position_msg_.layout.dim.clear();
		joint_position_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		joint_position_msg_.layout.dim[0].size = map_selected_joint_values_[ns_combo_->currentText()].size();
		joint_position_msg_.layout.dim[0].stride = 1;
		joint_position_msg_.layout.dim[0].label = "x_values"; // or whatever name you typically use to index vec1

		// copy in the data
		joint_position_msg_.data.clear();
		joint_position_msg_.data.insert(joint_position_msg_.data.end(), map_go_home_joint_values[ns_combo_->currentText()].begin(), map_go_home_joint_values[ns_combo_->currentText()].end());
		
		map_pub_joint_position_[ns_combo_->currentText()].publish(joint_position_msg_);
	}
	
	
	
	void JointPositionPlugin::sendPosition()
	{
		map_selected_joint_values_[ns_combo_->currentText()][0] = position_sliders_->slider_j0_->value();
		map_selected_joint_values_[ns_combo_->currentText()][1] = position_sliders_->slider_j1_->value();
		map_selected_joint_values_[ns_combo_->currentText()][2] = position_sliders_->slider_j2_->value();
		map_selected_joint_values_[ns_combo_->currentText()][3] = position_sliders_->slider_j3_->value();
		map_selected_joint_values_[ns_combo_->currentText()][4] = position_sliders_->slider_j4_->value();
		map_selected_joint_values_[ns_combo_->currentText()][5] = position_sliders_->slider_j5_->value();
		map_selected_joint_values_[ns_combo_->currentText()][6] = position_sliders_->slider_j6_->value();
			
		joint_position_msg_.layout.dim.clear();
		joint_position_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		joint_position_msg_.layout.dim[0].size = map_selected_joint_values_[ns_combo_->currentText()].size();
		joint_position_msg_.layout.dim[0].stride = 1;
		joint_position_msg_.layout.dim[0].label = "x_values"; // or whatever name you typically use to index vec1

		// copy in the data
		joint_position_msg_.data.clear();
		joint_position_msg_.data.insert(joint_position_msg_.data.end(), map_selected_joint_values_[ns_combo_->currentText()].begin(), map_selected_joint_values_[ns_combo_->currentText()].end());
		
		map_pub_joint_position_[ns_combo_->currentText()].publish(joint_position_msg_);
	}
	
	void JointPositionPlugin::resetSlidersPositions()
	{
		position_sliders_->updateSliders(map_current_joint_state_values_[ns_combo_->currentText()]);	
}
	
	void JointPositionPlugin::shutdownPlugin()
	{
		disconnect(timer_, SIGNAL(timeout()), this, SLOT(doUpdateCurves()));
		
		timer_->stop();
		
		shutdownROSComponents_();
		
		disconnect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
	
		disconnect(button_send_positions_, SIGNAL(pressed()), this, SLOT(sendPosition()));
			
		vlayout_positions_->removeWidget(position_sliders_);
		vlayout_positions_->removeWidget(button_send_positions_);
		
		vlayout_global_->removeWidget(ns_combo_);
		vlayout_global_->removeWidget(plot_checked_);
		
		delete plot_checked_;
		delete position_sliders_;
			
		delete button_send_positions_;
			
		delete ns_combo_;
			
		delete vlayout_global_;
		delete vlayout_positions_;
			
		delete widget_positions_;
		
		delete timer_;
	}
	
	void JointPositionPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
								qt_gui_cpp::Settings& instance_settings) const
	{
		// TODO save intrinsic configuration, usually using:
		// instance_settings.setValue(k, v)
	}

	void JointPositionPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
								   const qt_gui_cpp::Settings& instance_settings)
	{
		// TODO restore intrinsic configuration, usually using:
		// v = instance_settings.value(k)
	}
	
	void JointPositionPlugin::doUpdateCurves()
	{
		plot_checked_->updateAxisScale();
	}
	
	
	void JointPositionPlugin::jsCallback_panda_1_(const sensor_msgs::JointState::ConstPtr& msg)
	{
	
		QVector<double> values = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		
		bool zeros = std::all_of(values.begin(), values.end(), [](double position) { return position==0.0; });
		
		if (!zeros)
		{
				map_current_joint_state_values_["panda_1"] = values;
		}
		
		if (ns_combo_->currentText() == "panda_1")
		{
			if (!zeros)
			{
				double time = msg->header.stamp.sec + (msg->header.stamp.nsec/1e9);
			
				if (firstTime_ == 0)
				{
					firstTime_ = time;
				}

				double timeDuration = time - firstTime_;
			
				plot_checked_->updateDataCurves(values, timeDuration);
				
				if (map_sliders_is_init_["panda_1"]==false)
				{
					resetSlidersPositions();
					map_sliders_is_init_["panda_1"]=true;
				}
				
				position_sliders_->updateLabelJs(values);	
			}
		}
	}
	
	/*
	void JointPositionPlugin::jsCallback_panda_2_(const sensor_msgs::JointState::ConstPtr& msg)
	{	
		QVector<double> values = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		
		bool zeros = std::all_of(values.begin(), values.end(), [](double position) { return position==0.0; });
		
		if (!zeros)
		{
				map_current_joint_state_values_["panda_2"] = values;
		}
		
		if (ns_combo_->currentText() == "panda_2")
		{
			if (!zeros)
			{
				double time = msg->header.stamp.sec + (msg->header.stamp.nsec/1e9);
			
				if (firstTime_ == 0)
				{
					firstTime_ = time;
				}

				double timeDuration = time - firstTime_;
			
				plot_checked_->updateDataCurves(values, timeDuration);
				
				if (map_sliders_is_init_["panda_2"]==false)
				{
					resetSlidersPositions();
					map_sliders_is_init_["panda_2"]=true;
				}
				
				emit updateLabelJs(values);	
			}
		}
	}
	*/
	
	
	void JointPositionPlugin::setupROSComponents_()
	{
		QString name_of_position_controller = "joint_position_controller_ip";
		
		/* Setup publishers */
		map_pub_joint_position_.insert("panda_1",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
		//map_pub_joint_position_.insert("panda_2",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/panda_2/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
		
		/* Setup subscribers */
		map_sub_joint_handle_.insert("panda_1",getNodeHandle().subscribe(QString("/").append("joint_states").toStdString(), 100000, &JointPositionPlugin::jsCallback_panda_1_, this));
		//map_sub_joint_handle_.insert("panda_2",getNodeHandle().subscribe(QString("/panda_2/").append("joint_states").toStdString(), 100000, &JointPositionPlugin::jsCallback_panda_2_, this));
		
	}
	
	void JointPositionPlugin::shutdownROSComponents_()
	{
		map_pub_joint_position_["panda_1"].shutdown();
		//map_pub_joint_position_["panda_2"].shutdown();
		
		//map_sub_joint_handle_["panda_1"].shutdown();
		//map_sub_joint_handle_["panda_2"].shutdown();
		
	}
	
} // End of namespace

PLUGINLIB_EXPORT_CLASS(platform_panda_plugins_ns::JointPositionPlugin, rqt_gui_cpp::Plugin)


