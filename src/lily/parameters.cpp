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
    boundary_fitting_ = solverElement.attribute("boundary_fitting").toDouble();
    inner_fitting_ = solverElement.attribute("inner_fitting").toDouble();
    skel_smooth_ = solverElement.attribute("skel_smooth").toDouble();
    collision_ = solverElement.attribute("collision").toDouble();
    noise_p_ = solverElement.attribute("noise_p").toDouble();
    moving_ratio_ = solverElement.attribute("moving_ratio").toFloat();

    // boundary information
    QDomElement boundaryElement = root.firstChildElement(QString("boundary"));
    bin_num_ = boundaryElement.attribute("bin_num").toInt();
    knn_radius_ = boundaryElement.attribute("knn_radius").toFloat();
    noise_k_ = boundaryElement.attribute("noise_k").toInt();

    // segment information
    QDomElement segmentElement = root.firstChildElement(QString("segment"));
    segment_ratio_ = segmentElement.attribute("segment_ratio").toDouble();
    completion_degree_ = segmentElement.attribute("completion_degree").toInt();

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
    solverElement.setAttribute("boundary_fitting", boundary_fitting_);
    solverElement.setAttribute("inner_fitting", inner_fitting_);
    solverElement.setAttribute("skel_smooth", skel_smooth_);
    solverElement.setAttribute("collision", collision_);
    solverElement.setAttribute("noise_p", noise_p_);
    solverElement.setAttribute("moving_ratio", moving_ratio_);
    rootElement.appendChild(solverElement);

    // boundary information
    QDomElement boundaryElement = doc.createElement(QString("boundary"));
    boundaryElement.setAttribute("bin_num", bin_num_);
    boundaryElement.setAttribute("knn_radius", knn_radius_);
    boundaryElement.setAttribute("noise_k", noise_k_);
    rootElement.appendChild(boundaryElement);

    // segment information
    QDomElement segmentElement = doc.createElement(QString("segment"));
    segmentElement.setAttribute("segment_ratio", segment_ratio_);
    segmentElement.setAttribute("completion_degree", completion_degree_);
    rootElement.appendChild(segmentElement);

    QTextStream text_stream(&file);
    text_stream << doc.toString();
    file.close();

    return true;

}