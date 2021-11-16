/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef EXAMPLES_AHT_MAIN_DEVICE_H_
#define EXAMPLES_AHT_MAIN_DEVICE_H_

#include <string>
#include <map>

namespace hal
{

class device_t
{
public:
    sttaic

protected:
    std::string _name;
    uint32_t _id;

public:
    device_t(const std::string a_name, uint32_t a_num)
        : _parent(a_parent), _name(a_name), _id(a_id)
    {}

    virtual ~device_t();

    const std::string &name() { return _name; }

    uint32_t id() { return _id; }

    device_t &parent() { return *_parent; }

    std::map<std::string, >
};

} /* namespace hal */

#endif /* EXAMPLES_AHT_MAIN_DEVICE_H_ */
