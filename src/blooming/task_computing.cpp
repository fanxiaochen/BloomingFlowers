#include "lily.h"

//#include <core/cpd_nonrigid.hpp>
//
//#include "types_wrapper.h"
//#include "point_cloud.h"
//#include "task_computing.h"
//
//TaskController::TaskController(Task* task)
//	:QObject()
//{
//	task_ = task;
//
//	task_->moveToThread(&thread_);
//	connect(&thread_, SIGNAL(started()), task_, SLOT(run())); 
//    connect(task_, SIGNAL(finished()), &thread_, SLOT(quit()));
//    connect(&thread_, SIGNAL(finished()), this, SLOT(taskFinished()));
//    thread_.start();
//}
//
//TaskController::~TaskController()
//{
//
//}
//
//void TaskController::taskFinished()
//{
//	emit finished();
//}


//CPDReg::CPDReg()
//{}
//
//CPDReg::CPDReg(PointCloud* model, PointCloud* data)
//{
//	model_ = model;
//	data_ = data;
//}
//
//CPDReg::~CPDReg()
//{}
//
//void CPDReg::run()
//{
//	PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(*data_);
//	PointMatrix tracking_pm = POINTCLOUD_TO_MATRIX(*model_);
//
//	cpd::CPDNRigid<value_type, 3>* reg = new cpd::CPDNRigid<value_type, 3>();
//
//	reg->setInputData(tracking_pm, tracked_pm);
//	reg->setVision(false);
//	reg->setIterativeNumber(50);
//	reg->setVarianceTolerance(1e-5);
//	reg->setEnergyTolerance(1e-3);
//	reg->setOutlierWeight(0.1);
//	reg->setFgtFlag(true);
//	/*reg->setLowRankFlag(true);
//	reg->setKLowRank(10);*/
//	reg->run();
//
//	MATRIX_TO_POINTCLOUD(reg->getModel(), *model_);
//
//	return;
//}



