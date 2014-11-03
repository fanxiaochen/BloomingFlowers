//#ifndef TASK_COMPUTING_H
//#define TASK_COMPUTING_H
//
//#include <QObject>
//#include <QThread>
//
//class PointCloud;
//
//class Task: public QObject
//{
//	Q_OBJECT
//public:
//	Task();
//	virtual ~Task();
//
//public slots:
//	virtual void run() = 0;
//
//signals:
//	void finished();
//
//};
//
//class TaskController: public QObject
//{
//	Q_OBJECT
//
//public:
//	TaskController(Task* task);
//	virtual ~TaskController();
//
//	void start();
//
//public slots:
//	void taskFinished();
//
//signals:
//	void finished();
//
//private:
//	Task* task_;
//	QThread thread_;
//};

//class CPDReg: public Task
//{
//	Q_OBJECT
//
//public:
//	CPDReg();
//	CPDReg(PointCloud* model, PointCloud* data);
//	virtual ~CPDReg();
//
//public slots:
//	void run();
//
//private:
//	PointCloud* model_;
//	PointCloud* data_;
//};


#endif