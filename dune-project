(lang dune 3.17)
(using ctypes 0.3)

(name box2d)

(generate_opam_files true)

(source
 (github stevenjmoe/box2d-ocaml))

(authors "Steven Moe <steven.j.moe@proton.me>")
(maintainers "Steven Moe <steven.j.moe@proton.me>")

(license MIT)

(package
 (name box2d)
 (synopsis "OCaml bindings for Box2d")
 (description "OCaml bindings for Box2d")
 (depends ocaml ctypes ctypes-foreign))
