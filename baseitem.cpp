#include "baseitem.h"
#include <QQmlEngine>
#include <QtMath>
#include <QDebug>


SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
  _kalman_gain = _err_estimate / ( _err_estimate + _err_measure );
  _current_estimate = _last_estimate + _kalman_gain * ( mea - _last_estimate );
  _err_estimate = ( 1.0 - _kalman_gain ) * _err_estimate + fabs( _last_estimate - _current_estimate ) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q=q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError() {
  return _err_estimate;
}



BaseItem::BaseItem(QQuickItem *parent) : QQuickItem(parent)
{
    m_count = 0;
    m_avgReading = m_baseReading = QVector3D( 0.0f, 0.0f, 0.0f);
}

void BaseItem::registerTypes()
{
    qmlRegisterType<BaseItem>( "org.qtproject.example.navamessenger", 1, 0, "BaseItem" );
}

QVector3D BaseItem::axis() const
{
    return m_axis;
}

float BaseItem::angle() const
{
    return m_angle;
}

void BaseItem::setAxis(QVector3D axis)
{
    if (m_axis == axis)
        return;

    m_axis = axis;
    emit axisChanged(m_axis);
}

void BaseItem::setAngle(float angle)
{
    if (qFuzzyCompare(m_angle, angle))
        return;

    m_angle = angle;
    emit angleChanged(m_angle);
}

void BaseItem::calculate( float dx, float dy, float dz )
{
    QVector3D vecG = QVector3D( /*m_kf[0].updateEstimate(*/-dx/*)*/,
                                /*m_kf[1].updateEstimate(*/dy/*)*/, /*m_kf[2].updateEstimate(*/dz/*)*/ );
//    if( m_count < 5 )
//    {
//        ++m_count;
//        m_baseReading += vecG;
//    }else
//    {
//        m_avgReading = m_baseReading / m_count;
//        m_baseReading = QVector3D( 0.0f, 0.0f, 0.0f);
//        m_count = 0;
//    }
//    vecG = m_avgReading;
    vecG.normalize();


    float ax;
    float ay;
    float az;
    float an;
    QQuaternion::fromDirection( vecG, QVector3D( 0.0, 0.0, 1.0 ) ).getAxisAndAngle( &ax, &ay, &az, &an );

    setAxis( QVector3D( -ax, -ay, -az ) );
    setAngle( an );
}
