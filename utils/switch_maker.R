

con <- file("switchstatement.txt", open = "wt")
cat("switch (next_byte)\n{\n", file = con, append = FALSE)
close(con)
con <- file("switchstatement.txt", open = "at")

for (i in 0:255)
{
  cat(
    paste0("case ", sprintf("0x%02X", i), ":\nbreak;\n"),
    file = con,
    append = TRUE
  )
}

cat("}", file = con, append = TRUE)

close(con)

