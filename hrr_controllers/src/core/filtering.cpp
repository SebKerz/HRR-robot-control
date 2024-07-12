#include <hrr_controllers/core/filtering.h>

namespace hrr_controllers {

    VectorBuffer::VectorBuffer(const size_t &buf_size){
        m_x_buffer.resize(buf_size);
        m_y_buffer.resize(buf_size);
        m_z_buffer.resize(buf_size);
    };


    void VectorBuffer::add(const double x, const double y, const double z) {
        m_x_buffer.at(0) = x;
        m_y_buffer.at(0) = y;
        m_z_buffer.at(0) = z;
    }

    void VectorBuffer::add(const Eigen::Vector3d &pos){
        add(pos.x(), pos.y(), pos.z());
        if (m_cnt <= m_x_buffer.size())
            m_cnt += 1;
    }

    void VectorBuffer::rollBuffer() {
        for (auto i = getBufferSize() - 1; i > 0; i--) {
            m_x_buffer.at(i) = m_x_buffer.at(i - 1);
            m_y_buffer.at(i) = m_y_buffer.at(i - 1);
            m_z_buffer.at(i) = m_z_buffer.at(i - 1);
        }
    }

    void VectorBuffer::flush() {
        std::fill(m_x_buffer.begin(), m_x_buffer.end(), 0.);
        std::fill(m_y_buffer.begin(), m_y_buffer.end(), 0.);
        std::fill(m_z_buffer.begin(), m_z_buffer.end(), 0.);
    }

    void PoseBuffer::flush() {
        VectorBuffer::flush();
        std::fill(m_rot_buffer.begin(), m_rot_buffer.end(), Eigen::Matrix3d::Identity());
    }

    void VectorBuffer::setBufferSize(const size_t &size_new) {
        m_x_buffer.resize(size_new);
        m_y_buffer.resize(size_new);
        m_z_buffer.resize(size_new);
    }

    void PoseBuffer::setBufferSize(const size_t &size_new) {
        VectorBuffer::setBufferSize(size_new);
        m_rot_buffer.resize(size_new);
    }

    void PoseBuffer::add(const geometry_msgs::Pose &msg) {
        rollBuffer();
        VectorBuffer::add(msg.position.x, msg.position.y, msg.position.z);
        addRotation(msg.orientation);
    }

    void PoseBuffer::add(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat){
        rollBuffer();
        VectorBuffer::add(pos);
        m_rot_buffer.at(0) = quat.matrix();
    }

    void PoseBuffer::add(const geometry_msgs::Transform &msg) {
        rollBuffer();
        VectorBuffer::add(msg.translation.x, msg.translation.y, msg.translation.z);
        addRotation(msg.rotation);
    }

    void PoseBuffer::rollBuffer() {
        VectorBuffer::rollBuffer();
        for (auto i = getBufferSize() - 1; i > 0; i--) {
            m_rot_buffer.at(i) = m_rot_buffer.at(i - 1);
        }
    }

    void PoseBuffer::addRotation(const geometry_msgs::Quaternion &msg) {
        Eigen::Quaterniond q_tmp{};
        tf::quaternionMsgToEigen(msg, q_tmp);
        m_rot_buffer.at(0) = q_tmp.matrix();
    }

    SavGolFilter::SavGolFilter(const size_t &filter_size, const size_t &filter_order) :
            PoseBuffer{2 * filter_size + 1} {
        m_filter = std::make_shared<gram_sg::SavitzkyGolayFilter>(filter_size, -filter_size, filter_order, 0);
        m_deriv_filter = std::make_shared<gram_sg::SavitzkyGolayFilter>(filter_size, -filter_size, filter_order, 1);
        gram_sg::SavitzkyGolayFilterConfig sg_conf(50, 50, 2, 0);
        m_rot_filter = std::make_shared<gram_sg::RotationFilter>(sg_conf);
    }

    double SavGolFilter::filterData(const std::vector<double> &data) const{
        try{
            return m_filter->filter(data);
        } catch(...){
            return data.at(0);
        }
    }

    Eigen::Vector3d SavGolFilter::getPosition() const {
        auto[x_f, y_f, z_f] = getBuffers();
        try{
            if (isFilled()) {
                return Eigen::Vector3d{filterData(x_f), filterData(y_f), filterData(z_f)};
            }
        } catch ( ... ){};
        return Eigen::Vector3d{x_f.at(0), y_f.at(0), z_f.at(0)};
    }

    Eigen::Matrix3d SavGolFilter::getRotation() {
        if (isFilled()) {
            m_rot_filter->reset(Eigen::Matrix3d::Zero());
            for (auto &R: getRotBuffer()) {
                m_rot_filter->add(R);
            }
            return m_rot_filter->filter();
        } else {
            return getRotBuffer().at(0);
        }
    }


}