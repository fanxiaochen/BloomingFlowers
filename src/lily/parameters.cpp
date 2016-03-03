#include <QtXml>

#include "parameters.h"

Parameters::Parameters()
    :start_frame_(-1), end_frame_(-1), key_frame_(-1)
{}

Parameters::~Parameters()
{}

bool Parameters::load(const std::string& filename)
{
    QFile file(filename.c_str());
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }
    QDomDocument doc("blooming_flowers");
    if (!doc.setContent(&file)) {
        file.close();
        return false;
    }
    QDomElement root = doc.documentElement();


    // frame sequence information
    QDomElement sequenceElement = root.firstChildElement(QString("chosen_frame_sequence"));
    start_frame_ = sequenceElement.attribute("start_frame").toInt();
    end_frame_ = sequenceElement.attribute("end_frame").toInt();
    key_frame_ = sequenceElement.attribute("key_frame").toInt();

    // solver information
    QDomElement solverElement = root.firstChildElement(QString("solver"));
    iter_num_ = solverElement.attribute("iter_num").toInt();
    eps_ = solverElement.attribute("eps").toDouble();
    tip_fitting_ = solverElement.attribute("tip_fitting").toDouble();
    boundary_fitting_ = solverElement.attribute("boundary_fitting").toDouble();
    inner_fitting_ = solverElement.attribute("inner_fitting").toDouble();
    skel_smooth_ = solverElement.attribute("skel_smooth").toDouble();
    collision_ = solverElement.attribute("collision").toDouble();
    closure_ = solverElement.attribute("closure").toDouble();
    arap_ = solverElement.attribute("arap").toDouble();
    interpolate_ = solverElement.attribute("interpolate").toDouble();
    noise_p_ = solverElement.attribute("noise_p").toDouble();
    moving_ratio_ = solverElement.attribute("moving_ratio").toFloat();
    closure_ids_ = solverElement.attribute("closure_ids").toStdString();
    closure_start_frame_ = solverElement.attribute("closure_start_frame").toInt();

    // boundary information
    QDomElement boundaryElement = root.firstChildElement(QString("boundary"));
    bin_num_ = boundaryElement.attribute("bin_num").toInt();
    knn_radius_ = boundaryElement.attribute("knn_radius").toFloat();
    noise_k_ = boundaryElement.attribute("noise_k").toInt();
    min_boundary_ = boundaryElement.attribute("min_boundary").toInt();
    tip_radius_ = boundaryElement.attribute("tip_radius").toFloat();

    // segment information
    QDomElement segmentElement = root.firstChildElement(QString("segment"));
    segment_ratio_ = segmentElement.attribute("segment_ratio").toDouble();
    completion_degree_ = segmentElement.attribute("completion_degree").toInt();

    // petal order information
    QDomElement orderElement = root.firstChildElement(QString("order"));
    petal_order_ = orderElement.attribute("petal_order").toStdString();
    petal_num_ = orderElement.attribute("petal_num").toInt();

    // camera information
    QDomElement cameraElement = root.firstChildElement(QString("camera"));
    z_offset_ = cameraElement.attribute("z_offset").toFloat();

    file.close();
    return true;

}

bool Parameters::save(const std::string& filename)
{
    QFile file(filename.c_str());
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }

    QDomDocument doc("blooming_flowers");
    QDomProcessingInstruction xml_declaration = doc.createProcessingInstruction("xml", "version=\"1.0\"");
    doc.appendChild(xml_declaration);

    QDomElement rootElement = doc.createElement(QString("blooming_flowers"));
    doc.appendChild(rootElement);

    // frame sequence information
    QDomElement sequenceElement = doc.createElement(QString("chosen_frame_sequence"));
    sequenceElement.setAttribute("start_frame", start_frame_);
    sequenceElement.setAttribute("key_frame", key_frame_);
    sequenceElement.setAttribute("end_frame", end_frame_);
    rootElement.appendChild(sequenceElement);

    // solver information
    QDomElement solverElement = doc.createElement(QString("solver"));
    solverElement.setAttribute("iter_num", iter_num_);
    solverElement.setAttribute("eps", eps_);
    solverElement.setAttribute("tip_fitting", tip_fitting_);
    solverElement.setAttribute("boundary_fitting", boundary_fitting_);
    solverElement.setAttribute("inner_fitting", inner_fitting_);
    solverElement.setAttribute("skel_smooth", skel_smooth_);
    solverElement.setAttribute("collision", collision_);
    solverElement.setAttribute("closure", closure_);
    solverElement.setAttribute("arap", arap_);
    solverElement.setAttribute("interpolate", interpolate_);
    solverElement.setAttribute("noise_p", noise_p_);
    solverElement.setAttribute("moving_ratio", moving_ratio_);
    solverElement.setAttribute("closure_ids", QString(closure_ids_.c_str()));
    solverElement.setAttribute("closure_start_frame", closure_start_frame_);
    rootElement.appendChild(solverElement);

    // boundary information
    QDomElement boundaryElement = doc.createElement(QString("boundary"));
    boundaryElement.setAttribute("bin_num", bin_num_);
    boundaryElement.setAttribute("knn_radius", knn_radius_);
    boundaryElement.setAttribute("noise_k", noise_k_);
    boundaryElement.setAttribute("min_boundary", min_boundary_);
    boundaryElement.setAttribute("tip_radius", tip_radius_);
    rootElement.appendChild(boundaryElement);

    // segment information
    QDomElement segmentElement = doc.createElement(QString("segment"));
    segmentElement.setAttribute("segment_ratio", segment_ratio_);
    segmentElement.setAttribute("completion_degree", completion_degree_);
    rootElement.appendChild(segmentElement);

    // petal order
    QDomElement orderElement = doc.createElement(QString("order"));
    orderElement.setAttribute("petal_order", QString(petal_order_.c_str()));
    orderElement.setAttribute("petal_num", petal_num_);
    rootElement.appendChild(orderElement);

    // camera
    QDomElement cameraElement = doc.createElement(QString("camera"));
    cameraElement.setAttribute("z_offset", z_offset_);
    rootElement.appendChild(cameraElement);

    QTextStream text_stream(&file);
    text_stream << doc.toString();
    file.close();

    return true;

}


Eigen::MatrixXi Parameters::getPetalRelation()
{
    // for collision detection
    Eigen::MatrixXi petal_relation(petal_num_, petal_num_);
    // 0 means no relation between two petals; 
    // 1 means petal i should occludes petal j;
    // -1 means petal i should be occluded by petal j


    // not convenient to store, have to indicate manually here...


    // motion transfer for heart lily
    petal_relation << 
        0, 1, -1, -1, -1, -1,
        -1, 0, -1, -1, -1, -1,
        1, 1, 0, -1, -1, -1,
        1, 1, 1, 0, 1 ,-1,
        1, 1, 1, -1, 0, -1,
        1, 1, 1, 1, 1, 0;

    //// lily
    //petal_relation << 
    //    0, 1, -1, -1, -1, -1,
    //    -1, 0, -1, -1, -1, -1,
    //    1, 1, 0, -1, -1, -1,
    //    1, 1, 1, 0, 0 ,0,
    //    1, 1, 1, 0, 0, 0,
    //    1, 1, 0, 0, 0, 0;

    //  // orchid
    //  petal_relation << 
    //0, 1, -1, -1, 0,
    //-1, 0, -1, 0, -1,
    //1, 1, 0, 1, 1,
    //1, 0, -1, 0 ,-1,
    //0, 1, -1, 1, 0;

    //// golden lily
    //petal_relation << 
    //    0,  1,  1,  1,  1,  1,
    //    -1, 0, -1,  1, -1, -1,
    //    -1,  1,  0,  1,  -1,  1,
    //    -1,-1, -1,  0, -1,  1,
    //    -1,  1,  1,  1,  0,  1,
    //    -1, 1, -1, -1, -1,  0;


    //// waterlily
    //petal_relation.setZero();
    //
    //// a
    //petal_relation(0, 1) = 1;
    //petal_relation(0, 2) = -1;
    //petal_relation(0, 8) = 1;
    //petal_relation(0, 9) = -1;
    //petal_relation(0, 11) = -1;
    //petal_relation(0, 12) = -1;
    //petal_relation(0, 13) = 1;

    //// b
    //petal_relation(1, 2) = -1;
    //petal_relation(1, 3) = 1;

    //// c
    //petal_relation(2, 3) = 1;
    //petal_relation(2, 4) = 1;

    //// d
    //petal_relation(3, 4) = -1;
    //petal_relation(3, 5) = -1;
    //petal_relation(3, 6) = -1;

    //// e
    //petal_relation(4, 5) = 1;
    //petal_relation(4, 6) = 1;

    //// f 
    //petal_relation(5, 6) = -1;
    //petal_relation(5, 7) = -1;
    //petal_relation(5, 8) = -1;
    //petal_relation(5, 9) = 1;

    //// g
    //petal_relation(6, 7) = -1;
    //petal_relation(6, 8) = -1;
    //
    //// h
    //petal_relation(7, 8) = 1;
    //petal_relation(7, 9) = 1;

    //// i
    //petal_relation(8, 9) = -1;
    //petal_relation(8, 10) = 1;

    //// j
    //petal_relation(9, 10) = 1;
    //petal_relation(9, 11) = -1;

    //// k
    //petal_relation(10, 11) = -1;
    //petal_relation(10, 12) = -1;

    //// l
    //petal_relation(11, 12) = -1;
    //petal_relation(11, 13) = 1;

    //// m
    //petal_relation(12, 13) = 1;
    //petal_relation(12, 2) = 1;

    //Eigen::MatrixXi r1 = petal_relation;
    //Eigen::MatrixXi r2 = -petal_relation.transpose();
    //petal_relation = r1 + r2;

   //// std::cout << petal_relation << std::endl;

    return petal_relation;
}