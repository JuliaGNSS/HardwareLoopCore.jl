using Documenter
using HardwareLoopCore

makedocs(
    sitename = "HardwareLoopCore.jl",
    modules = [HardwareLoopCore],
    authors = "JuliaGNSS",
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical = "https://JuliaGNSS.github.io/HardwareLoopCore.jl",
    ),
    pages = [
        "Home" => "index.md",
        "Usage" => "usage.md",
        "Writing a driver" => "drivers.md",
        "API Reference" => "api.md",
    ],
    # Every exported symbol must appear in the manual, and every `@ref` must
    # resolve: both are build errors, not warnings.
    checkdocs = :exports,
)

deploydocs(
    repo = "github.com/JuliaGNSS/HardwareLoopCore.jl.git",
    devbranch = "main",
    push_preview = true,
)
