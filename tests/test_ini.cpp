#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "check.hpp"
#include "core/ini.hpp"

using namespace rallysim;

namespace
{
    ini_file parse(const std::string &text)
    {
        std::istringstream in(text);
        return parse_ini(in, "t.ini");
    }

    // The messages parsing `text` throws, or none if it parses.
    std::vector<std::string> errors_of(const std::string &text)
    {
        try
        {
            parse(text);
        }
        catch (const ini_error &e)
        {
            return e.messages;
        }
        return {};
    }

    // Exactly one error, at `line`, whose message holds `words`.
    void check_one_error(const char *what, const std::string &text, int line, const std::string &words)
    {
        const std::vector<std::string> e = errors_of(text);
        const std::string at = "t.ini:" + std::to_string(line) + ": ";
        const bool ok = e.size() == 1 && e[0].rfind(at, 0) == 0 && e[0].find(words) != std::string::npos;
        if (!ok)
            std::printf("  got %zu error(s)%s%s\n", e.size(), e.empty() ? "" : ", first: ", e.empty() ? "" : e[0].c_str());
        check_true(what, ok);
    }

    const ini_entry *entry(const ini_file &f, const std::string &section, const std::string &key)
    {
        for (const ini_section &s : f.sections)
            if (s.name == section)
                for (const ini_entry &e : s.entries)
                    if (e.key == key)
                        return &e;
        return nullptr;
    }

    bool has(const ini_file &f, const std::string &section, const std::string &key, const std::string &value, int line)
    {
        const ini_entry *e = entry(f, section, key);
        return e && e->value == value && e->line == line;
    }
}

int main()
{
    // ---- what a good file gives ------------------------------------------------------------------------------
    {
        const ini_file f = parse("# a comment line\n"
                                 "\n"
                                 "[run]\n"
                                 "duration = 10 s\n"
                                 "   dt=0.5 ms   \n"          // spaces around the line and the '=' do not matter
                                 "\t[car]\t\n"
                                 "mass = 1.25 t   # trailing comment\n"
                                 "description = a = b\n"      // the value is everything after the first '='
                                 "empty =\n");
        check_true("ini: two sections, in order", f.sections.size() == 2 && f.sections[0].name == "run" &&
                                                     f.sections[1].name == "car");
        check_true("ini: section line", f.sections[1].line == 6);
        check_true("ini: entry", has(f, "run", "duration", "10 s", 4));
        check_true("ini: spaces trimmed", has(f, "run", "dt", "0.5 ms", 5));
        check_true("ini: comment cut", has(f, "car", "mass", "1.25 t", 7));
        check_true("ini: split at the first '='", has(f, "car", "description", "a = b", 8));
        check_true("ini: an empty value is kept (its type decides later)", has(f, "car", "empty", "", 9));
        check_true("ini: source kept", f.source == "t.ini");
    }
    {
        // Windows line ends and a byte-order mark
        const ini_file f = parse("\xEF\xBB\xBF[run]\r\nduration = 10\r\n");
        check_true("ini: BOM and CRLF", f.sections.size() == 1 && has(f, "run", "duration", "10", 2));
    }
    {
        // a value ending in ',' continues, with a comment per point, and keeps its first line
        const ini_file f = parse("[driver]\n"
                                 "steer = 0 @ 0,        # straight\n"
                                 "\n"
                                 "        0 @ 2.6 s,    # turn in\n"
                                 "        18 deg @ 3 s\n"
                                 "handbrake = 0\n");
        check_true("ini: continued value", has(f, "driver", "steer", "0 @ 0, 0 @ 2.6 s, 18 deg @ 3 s", 2));
        check_true("ini: entry after a continued value", has(f, "driver", "handbrake", "0", 6));
    }
    {
        // an empty file and a file of only comments are fine: layer 3 says what is missing
        check_true("ini: empty file", parse("").sections.empty());
        check_true("ini: only comments", parse("# nothing\n\n   # here\n").sections.empty());
        check_true("ini: an empty section", parse("[run]\n").sections.size() == 1);
    }

    // ---- each error, on its line -----------------------------------------------------------------------------
    check_one_error("ini: entry before any section", "mass = 1\n[car]\n", 1, "before any [section]");
    check_one_error("ini: no '='", "[car]\nmass 1200\n", 2, "expected [section] or key = value");
    check_one_error("ini: empty key", "[car]\n = 3\n", 2, "empty key");
    check_one_error("ini: key with a space", "[car]\nspring front = 1\n", 2, "invalid key");
    check_one_error("ini: key in capitals", "[car]\nMass = 1\n", 2, "invalid key");
    check_one_error("ini: key with a dot", "[car]\ncar.mass = 1\n", 2, "invalid key");
    check_one_error("ini: unclosed header", "[run\n", 1, "malformed section header");
    check_one_error("ini: empty section name", "[]\n", 1, "invalid section name");
    check_one_error("ini: section name in capitals", "[Car]\n", 1, "invalid section name");
    check_one_error("ini: duplicate key", "[car]\nmass = 1\nlf = 2\nmass = 3\n", 4, "first on line 2");
    check_one_error("ini: duplicate section", "[car]\nmass = 1\n[run]\n[car]\nmass = 2\n", 4,
                    "duplicate section [car] (first on line 1)");
    check_one_error("ini: unterminated continuation", "[driver]\nsteer = 0 @ 0,\n# end\n", 2, "unterminated");
    // a line ending in ']' is not a header unless it starts with '['
    check_one_error("ini: a value ending in ']' is an entry", "[start]\nposition = (1, 2]\nheading\n", 3,
                    "expected [section] or key = value");

    // ---- mistakes that must not pass quietly ------------------------------------------------------------------
    {
        // after a bad header, its entries go nowhere (not into the section before it), and are not reported again
        const std::string text = "[run]\nduration = 10\n[Car]\nmass = 1250\nlf = 1.3\n[driver]\nsteer = 0\n";
        check_one_error("ini: entries under a bad header are dropped", text, 3, "invalid section name");
        // were they taken into [run], this duration would be a second, misleading error: a duplicate in [run]
        check_one_error("ini: no false duplicate from a bad header", "[run]\nduration = 10\n[Run 2]\nduration = 5\n", 3,
                        "invalid section name");
        std::istringstream in("[run]\nduration = 10\n[car]\nmass = 1250\n");
        const ini_file good = parse_ini(in, "t.ini");
        check_true("ini: (the same file with a good header keeps them)", has(good, "car", "mass", "1250", 4));
    }
    {
        // a stray ',' before a new entry: reported on the entry with the ',', and the new entry still read
        const std::string text = "[driver]\nsteer = 0 @ 0, 18 deg @ 3 s,\nhandbrake = 0\n";
        check_one_error("ini: stray ',' before a key", text, 2, "'steer' ends with ',' but line 3 starts a new entry");
        check_one_error("ini: stray ',' before a section", "[driver]\nsteer = 0,\n[run]\n", 2, "line 3 starts a new entry");
    }
    {
        // every error in the file at once, in order
        const std::vector<std::string> e = errors_of("x = 1\n[car]\nmass\nmass = 1\nmass = 2\n[Bad]\n");
        check_true("ini: every error, in line order", e.size() == 4 && e[0].rfind("t.ini:1:", 0) == 0 &&
                                                         e[1].rfind("t.ini:3:", 0) == 0 && e[2].rfind("t.ini:5:", 0) == 0 &&
                                                         e[3].rfind("t.ini:6:", 0) == 0);
        try
        {
            parse("x = 1\ny = 2\n");
        }
        catch (const std::exception &ex) // main() catches it as any exception and prints what()
        {
            check_true("ini: what() lists them all", std::string(ex.what()) == "t.ini:1: 'x = 1' comes before any [section]\n"
                                                                              "t.ini:2: 'y = 2' comes before any [section]\n");
        }
    }
    {
        // past 20, the rest are counted, not listed
        std::string text;
        for (int k = 0; k < 30; ++k)
            text += "bad line\n";
        const std::vector<std::string> e = errors_of(text);
        check_true("ini: at most 20 errors, then a note", e.size() == 21 && e.back() == "t.ini: more errors not shown");
    }

    // ---- the reference scenario reads at this layer --------------------------------------------------------
    {
        std::ifstream file(RALLYSIM_SOURCE_DIR "/scenarios/fw_showcase.ini");
        check_true("ini: fw_showcase.ini opens", file.is_open());
        const ini_file f = parse_ini(file, "fw_showcase.ini");
        check_true("ini: fw_showcase has its five sections", f.sections.size() == 5);
        check_true("ini: fw_showcase's continued hold_speed",
                   has(f, "driver", "hold_speed",
                       "50 km/h @ 0, 50 km/h @ 1.5 s, 30 km/h @ 2.5 s, 30 km/h @ 6 s, 45 km/h @ 9 s",
                       entry(f, "driver", "hold_speed") ? entry(f, "driver", "hold_speed")->line : -1));
        check_true("ini: fw_showcase's description stops at its comment",
                   has(f, "scenario", "description", "Crest jump into a handbrake hairpin, on gravel",
                       entry(f, "scenario", "description") ? entry(f, "scenario", "description")->line : -1));
    }

    return finish("ini");
}
