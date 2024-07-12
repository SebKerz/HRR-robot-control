/*
 * This file requires the savitky-golay filter library to be built on the current system.
 * Please follow the instructions on GitHub
 * https://github.com/arntanguy/gram_savitzky_golay
 */
#ifndef HRR_CONTROLLERS_FILTERING_H
#define HRR_CONTROLLERS_FILTERING_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include <gram_savitzky_golay/spatial_filters.h>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

namespace hrr_controllers{


    class VectorBuffer{
    public:
        explicit VectorBuffer(const size_t &buf_size);

        ~VectorBuffer() = default;

        [[nodiscard]] size_t getBufferSize() const{ return m_x_buffer.size();}

        virtual void setBufferSize(const size_t &size_new);

        [[nodiscard]] std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getBuffers() const{
            return std::make_tuple(m_x_buffer, m_y_buffer, m_z_buffer);
        }

        [[nodiscard]] bool isFilled() const{ return m_cnt >= m_x_buffer.size();};

        void add(const double x, const double y, const double z);

        void add(const Eigen::Vector3d &pos);

        virtual void flush();
    protected:
        std::vector<double> m_x_buffer{};
        std::vector<double> m_y_buffer{};
        std::vector<double> m_z_buffer{};

        virtual void rollBuffer();

    private:

        size_t m_cnt{0};

    };


    class PoseBuffer : public VectorBuffer {
    public:
        explicit PoseBuffer(const size_t &buf_size): VectorBuffer{buf_size}{ m_rot_buffer.resize(buf_size);}

        void setBufferSize(const size_t &size_new) final;

        [[nodiscard]] std::vector<Eigen::Matrix3d> getRotBuffer() const{
            return m_rot_buffer;
        }

        void add(const geometry_msgs::Pose &msg);

        void add(const geometry_msgs::Transform &msg);

        void add(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat);

        void flush() final;

    private:

        std::vector<Eigen::Matrix3d> m_rot_buffer{};

        void rollBuffer() final;

        void addRotation(const geometry_msgs::Quaternion &msg);
    };


    class SavGolFilter: public PoseBuffer{

    public:
        SavGolFilter(const size_t &filter_size, const size_t &filter_order);

        ~SavGolFilter() = default;

        [[nodiscard]] double filterData(const std::vector<double> &data) const;

        [[nodiscard]] Eigen::Vector3d getPosition() const;

        [[nodiscard]] Eigen::Matrix3d getRotation();

    private:
        std::shared_ptr<gram_sg::SavitzkyGolayFilter> m_filter, m_deriv_filter;
        std::shared_ptr<gram_sg::RotationFilter> m_rot_filter;
    };

}

#endif


