use std::{borrow::Cow, fs::read_to_string, process::ExitCode};

use regex::{Captures, Regex};

fn main() -> ExitCode {
    let args = std::env::args().collect::<Vec<_>>();
    let input_path = args
        .get(1)
        .expect("Expected 1 argument: the path to the file to parse.");
    let file: &str = &read_to_string(input_path)
        .unwrap_or_else(|_| panic!("Could not read file at input {}", input_path));

    let result = injected(file, |file_path| {
        let mut path = "..".to_string();
        path.push_str(file_path);
        path.to_string()
    });
    match result {
        Ok(result) => {
            print!("{result}");
        }
        Err(error) => {
            for e in error.errors {
                match e {
                    ErrorType::IncorrectTag => {
                        eprintln!("❌ ERROR: a <load> tag was incorrectly written in {}, maybe missing path or marker, or not using `'` for string delimiters ?", input_path);
                    }
                    ErrorType::IncorrectPath(path) => {
                        eprintln!("❌ ERROR: path {} not found", path);
                    }
                    ErrorType::IncorrectMarker(IncorrectMarker { marker, filepath }) => {
                        eprintln!("❌ ERROR: marker {} not found in {}", marker, filepath);
                    }
                }
            }
            if let Some(result) = error.result {
                print!("{result}");
            }
            return ExitCode::from(1);
        }
    }
    ExitCode::from(0)
}

#[derive(Debug, PartialEq)]
pub struct IncorrectMarker {
    pub marker: String,
    pub filepath: String,
}

#[derive(Debug, PartialEq)]
pub enum ErrorType {
    IncorrectTag,
    IncorrectPath(String),
    IncorrectMarker(IncorrectMarker),
}
#[derive(Debug)]
pub struct InjectError<'a> {
    /// Contains the best result we could do.
    pub result: Option<Cow<'a, str>>,
    /// Errors encountered
    pub errors: Vec<ErrorType>,
}

fn injected(source_text: &str, get_path: fn(&str) -> String) -> Result<String, InjectError> {
    let source_text = &source_text.replace("\r\n", "\n");
    let re = Regex::new(r"<load.*>").unwrap();
    let total_to_inject = re.find_iter(source_text).count();
    let mut injected_count = 0;
    // Regex to find "<load" tags and capture their info (path + marker)
    let re = Regex::new(r"<load path='(.*)'.*marker='(.*)'.*>\n?").unwrap();

    let mut error = InjectError {
        result: None,
        errors: Vec::new(),
    };
    let result = re.replace_all(source_text, |caps: &Captures| {
        let infos = &caps.extract::<2>().1;
        assert!(
            infos.len() == 2,
            "load tag encountered without a path or marker"
        );
        let path = get_path(infos[0]);
        // Reading file from the path of the tag of input file
        let Ok(to_inject) = read_to_string(&path) else {
            error.errors.push(ErrorType::IncorrectPath(path));
            return "".to_string();
        };
        let to_inject = to_inject.replace("\r\n", "\n");
        // Regex to find the markers inside comments, and only print what's inside
        // FIXME: I think we should just paste all the inside,
        // and then remove all "// DOCUSAURUS*"" lines, to allow reuse of a same file.
        let regex = format!(
            r"// DOCUSAURUS: {} start\n((?:\s|.)*)\s+\/\/ DOCUSAURUS: {} stop",
            infos[1], infos[1]
        );
        let re = Regex::new(&regex).unwrap();
        let to_keep = re
            .captures_iter(&to_inject)
            .map(|c| {
                let to_keep = c.extract::<1>();
                injected_count += 1;
                let injected_string = to_keep.1[0];
                let injected_string = remove_indent(injected_string)
                    .unwrap_or(injected_string.to_string())
                    .trim_end()
                    .to_string();
                injected_string
            })
            .collect::<Vec<_>>();
        if to_keep.is_empty() {
            error
                .errors
                .push(ErrorType::IncorrectMarker(IncorrectMarker {
                    marker: infos[1].to_string(),
                    filepath: infos[0].to_string(),
                }));
        }
        let mut result = to_keep.join("");
        result.push('\n');
        result
    });
    if (injected_count + error.errors.len()) != total_to_inject {
        error.errors.push(ErrorType::IncorrectTag);
    }
    if !error.errors.is_empty() {
        return Err(error);
    }
    let re = Regex::new(r"(.*\/\/ DOCUSAURUS:.*\n)").unwrap();
    let result = re.replace_all(&result, |_: &Captures| "").to_string();
    Ok(result)
}

fn remove_indent(source: &str) -> Option<String> {
    let source = source.replace("\r\n", "\n");
    let min_indent = source
        .lines()
        .filter_map(|l| {
            // Don't count empty lines
            if !l.chars().any(|c| !c.is_whitespace()) {
                return None;
            }
            Some(l.chars().take_while(|c| c.is_whitespace()).count())
        })
        .min()?;
    if min_indent == 0 {
        return Some(source.to_string());
    }
    let unindented_lines = source
        .lines()
        .map(|l| l.chars().skip(min_indent).collect::<String>());
    let mut result = String::new();
    for unindented_line in unindented_lines {
        result.push_str(&unindented_line);
        result.push('\n');
    }
    Some(result)
}

#[test]
fn simple_injection() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker='ToInject1_1' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect("This should not error out").trim_end(),
        "correct data1 1"
    );
}

#[test]
fn marker_short() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker='ToInject' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect("This should not error out").trim_end(),
        "correct data short"
    );
}

#[test]
fn nest_removal() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker='ToInject1_nest' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect("This should not error out"),
        "correct data nest1
correct data nested
correct data nest2
"
    );
}

#[test]
fn indent_removal_simple() {
    use crate::*;

    let result = remove_indent(
        "    correct data indented 1
        correct data indented more
    correct data indented 2
",
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect("This should not error out"),
        "correct data indented 1
    correct data indented more
correct data indented 2
"
    );
}

#[test]
fn indent_removal() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker='ToInject1_indent' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect("This should not error out").trim_end(),
        "correct data not indented
    empty line without space next
        empty line without space next

correct data not indented again"
    );
}

#[test]
fn simple_tag_error() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker=\"ToInject1_1\" />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert!(matches!(
        result.expect_err("This should error out").errors[0],
        ErrorType::IncorrectTag
    ));
}

#[test]
fn simple_path_error() {
    use crate::*;

    let result = injected(
        "<load path='this_path_does_not_exist_obviously' marker='ToInject1_wrong' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    assert_eq!(
        result.expect_err("This should error out").errors[0],
        ErrorType::IncorrectPath("this_path_does_not_exist_obviously".to_string())
    );
}

#[test]
fn simple_marker_error() {
    use crate::*;

    let result = injected(
        "<load path='test/to_inject1.txt' marker='ToInject1_wrong' />",
        |path| path.to_string(),
    );

    // Trimming end for cross platform, on windows I had \r finishing result.
    match &result.expect_err("This should error out").errors[0] {
        ErrorType::IncorrectMarker(_) => {}
        invalid_value => {
            panic!("unexpected error type: {:?}", invalid_value);
        }
    }
}
