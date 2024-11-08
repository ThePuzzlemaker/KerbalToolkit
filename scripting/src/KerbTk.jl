module KerbTk

import REPL

greet() = print("Hello World!")

ktk_jlrepl_inbuf  = undef
ktk_jlrepl_outbuf = undef

function run_repl()
    inbuf = open(Libc.dup(RawFD(ktk_jlrepl_inbuf)))
    outbuf = open(Libc.dup(RawFD(ktk_jlrepl_outbuf)))

    ENV["TERM"] = "alacritty"
    term = REPL.Terminals.TTYTerminal("alacritty", inbuf, outbuf, outbuf)
    repl = REPL.LineEditREPL(term, true, true)
    REPL.run_repl(repl)
end

end # module KerbTk
