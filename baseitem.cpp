#include "baseitem.h"
#include <QQmlEngine>
#include <QtMath>
#include <QDebug>


KalmanFilter::KalmanFilter(float measurementError, float estimationError, float rate)
{
  m_measurementError = measurementError;
  m_estimationError = estimationError;
  m_rate = rate;
}

float KalmanFilter::updateEstimate(float measurement)
{
  m_kalmanGain = m_estimationError / ( m_estimationError + m_measurementError );
  m_currentEstimation = m_lastEstimation + m_kalmanGain * ( measurement - m_lastEstimation );
  m_estimationError = ( 1.0 - m_kalmanGain ) * m_estimationError + fabs( m_lastEstimation - m_currentEstimation ) * m_rate;
  m_lastEstimation = m_currentEstimation;

  return m_currentEstimation;
}

void KalmanFilter::setMeasurementError(float measurementError)
{
  m_measurementError = measurementError;
}

void KalmanFilter::setEstimationError(float estimationError)
{
  m_estimationError = estimationError;
}

void KalmanFilter::setProcessNoise(float rate)
{
  m_rate = rate;
}

float KalmanFilter::getKalmanGain()
{
  return m_kalmanGain;
}

float KalmanFilter::getEstimationError()
{
  return m_estimationError;
}



BaseItem::BaseItem(QQuickItem *parent) : QQuickItem(parent)
{

}

void BaseItem::registerTypes()
{
    qmlRegisterType<BaseItem>( "org.tdevelopers.sensorfusion", 1, 0, "BaseItem" );
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
    m_currentRotation = QVector3D( m_kf[0].updateEstimate(-dx),
                                m_kf[1].updateEstimate(dy),
                                m_kf[2].updateEstimate(dz) );
    QVector3D vecG = m_currentRotation - m_baseRotation;
    vecG.normalize();

    float ax;
    float ay;
    float az;
    float an;
    QQuaternion::fromDirection(
                vecG, QVector3D( 0.0, 0.0, 1.0 ).normalized() )
            .getAxisAndAngle( &ax, &ay, &az, &an );

    setAxis( QVector3D( -ax, -ay, -az ) );
    setAngle( an );
}

void BaseItem::resetRotation()
{
    m_baseRotation = m_currentRotation;
}
