#pragma once

#include <fmt/format.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

template<>
struct fmt::formatter<boost::uuids::uuid>{

    constexpr auto parse(format_parse_context& ctx) {
        auto it = ctx.begin();
        it++;
        return it;
    }

    template <typename FormatContext>
    auto format(const boost::uuids::uuid& uuid, FormatContext& ctx) {
        return format_to(ctx.out(), "{}", to_string(uuid));
    }
};