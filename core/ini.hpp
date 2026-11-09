#pragma once
#include <algorithm>
#include <istream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rallysim
{
    // Layer 1 of the scenario files (docs/implementation.md): the lines of an .ini file, with no meaning attached.

    // Key value pair from each line
    // e.g. mass = 1.25 t
    // key == "mass"; value = "1.25 t"
    struct ini_entry
    {
        std::string key, value;
        int line; // where the entry starts (a continued value keeps its first line)
    };

    // Group of entries belonging to the same section
    // Started with [name], line is the line in which it starts
    struct ini_section
    {
        std::string name;
        int line;
        std::vector<ini_entry> entries;
    };

    // Group of sections, source is the filename/path
    struct ini_file
    {
        std::string source;
        std::vector<ini_section> sections;
    };

    // Every problem found in a file, each as "source:line: message", in file order. what() gives them all, one per line.
    struct ini_error : std::runtime_error
    {
        std::vector<std::string> messages;

        explicit ini_error(std::vector<std::string> m)
            : std::runtime_error(join(m)), messages(std::move(m)) {}

    private:
        static std::string join(const std::vector<std::string> &m)
        {
            std::string all;
            for (const std::string &s : m)
                all += s + "\n";
            return all;
        }
    };

    // A section or key name: lower case letters, digits and '_', not empty.
    inline bool is_ini_name(const std::string &s)
    {
        return !s.empty() && s.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") == std::string::npos;
    }

    // Reads the lines of `in` (see docs/implementation.md, section 1). Throws ini_error with every problem (up to 20)
    // if any line is malformed; otherwise every entry is in its section, in file order.
    inline ini_file parse_ini(std::istream &in, const std::string &source)
    {
        constexpr std::size_t kMaxErrors = 20;

        ini_file input;
        input.source = source;

        std::vector<std::string> errors;
        bool too_many = false;

        int number = 0;
        int current_section = -1;
        bool skipping = false; // inside a section whose header was bad: its entries are not taken (or reported)

        // Entry currently waiting for a continuation line.
        int pending_entry = -1;
        int pending_section = -1;

        auto error = [&](int at, const std::string &message)
        {
            if (errors.size() < kMaxErrors)
                errors.push_back(source + ":" + std::to_string(at) + ": " + message);
            else
                too_many = true;
        };

        // Remove whitespace from both ends of a string.
        auto trim = [](std::string &s)
        {
            const auto first = s.find_first_not_of(" \t\r\n");

            if (first == std::string::npos)
            {
                s.clear();
                return;
            }

            const auto last = s.find_last_not_of(" \t\r\n");
            s = s.substr(first, last - first + 1);
        };

        std::string line;

        while (std::getline(in, line))
        {
            ++number;

            // Some editors start a UTF-8 file with a byte-order mark: three invisible bytes.
            if (number == 1 && line.rfind("\xEF\xBB\xBF", 0) == 0)
                line.erase(0, 3);

            // Remove comments.
            const auto comment = line.find('#');
            if (comment != std::string::npos)
                line.erase(comment);

            trim(line);

            // Ignore blank lines and comment-only lines.
            if (line.empty())
                continue;

            // Handle a continuation before interpreting the line normally. A line that starts a section or holds
            // an '=' is not part of a value (no value has either): the ',' before it was a stray one, so report it
            // and read the line as what it is.
            if (pending_entry != -1)
            {
                auto &entry = input.sections[pending_section].entries[pending_entry];

                if (line.front() == '[' || line.find('=') != std::string::npos)
                {
                    error(entry.line, "'" + entry.key + "' ends with ',' but line " + std::to_string(number) +
                                          " starts a new entry");
                    pending_entry = -1;
                    pending_section = -1;
                }
                else
                {
                    entry.value += " " + line;

                    if (entry.value.back() != ',')
                    {
                        pending_entry = -1;
                        pending_section = -1;
                    }

                    continue;
                }
            }

            // Section header.
            if (line.front() == '[')
            {
                if (line.back() != ']')
                {
                    error(number, "malformed section header: '" + line + "'");
                    skipping = true;
                    continue;
                }

                std::string name = line.substr(1, line.size() - 2);
                trim(name);

                if (!is_ini_name(name))
                {
                    error(number, "invalid section name: '" + name + "' (lower case letters, digits and _)");
                    skipping = true;
                    continue;
                }

                const auto same = std::find_if(input.sections.begin(), input.sections.end(),
                                               [&](const ini_section &s) { return s.name == name; });
                if (same != input.sections.end())
                {
                    error(number, "duplicate section [" + name + "] (first on line " + std::to_string(same->line) + ")");
                    skipping = true;
                    continue;
                }

                input.sections.push_back({name, number, {}});
                current_section = static_cast<int>(input.sections.size()) - 1;
                skipping = false;

                continue;
            }

            // Entries of a section whose header was bad: that header is already reported.
            if (skipping)
                continue;

            // Key-value entry.
            const auto equal = line.find('=');

            if (equal == std::string::npos)
            {
                error(number, "expected [section] or key = value, got '" + line + "'");
                continue;
            }

            if (current_section == -1)
            {
                error(number, "'" + line + "' comes before any [section]");
                continue;
            }

            std::string key = line.substr(0, equal);
            std::string value = line.substr(equal + 1);

            trim(key);
            trim(value);

            if (key.empty())
            {
                error(number, "empty key");
                continue;
            }

            if (!is_ini_name(key))
            {
                error(number, "invalid key: '" + key + "' (lower case letters, digits and _)");
                continue;
            }

            auto &section = input.sections[current_section];

            // Check for duplicate keys in this section.
            auto duplicate = std::find_if(
                section.entries.begin(),
                section.entries.end(),
                [&](const ini_entry &entry)
                {
                    return entry.key == key;
                });

            if (duplicate != section.entries.end())
            {
                error(number, "duplicate key '" + key +
                                  "' (first on line " + std::to_string(duplicate->line) + ")");
                continue;
            }

            section.entries.push_back({key, value, number});

            // A trailing comma means the value continues on the next non-empty line.
            if (!value.empty() && value.back() == ',')
            {
                pending_section = current_section;
                pending_entry = static_cast<int>(section.entries.size()) - 1;
            }
        }

        // A continuation cannot be left unfinished at EOF.
        if (pending_entry != -1)
        {
            const auto &entry =
                input.sections[pending_section].entries[pending_entry];

            error(entry.line, "unterminated continuation for key '" + entry.key + "'");
        }

        if (too_many)
            errors.push_back(source + ": more errors not shown");

        if (!errors.empty())
            throw ini_error(std::move(errors));

        return input;
    }

}
