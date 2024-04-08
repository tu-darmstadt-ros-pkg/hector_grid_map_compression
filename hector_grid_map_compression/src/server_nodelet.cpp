#include <nodelet/nodelet.h>
#include <hector_grid_map_compression/server.h>

namespace hector_grid_map_compression
{

class CompressionNodelet : public nodelet::Nodelet
{
    protected:
        virtual void onInit()
        {
            server_ = std::make_shared<Server>(getNodeHandle(), getPrivateNodeHandle());
        }

        std::shared_ptr<Server> server_;
};
    
}  // namespace hector_grid_map_compression

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_grid_map_compression::CompressionNodelet, nodelet::Nodelet);
