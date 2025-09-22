fn main() {
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");

    include_images();
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                "_defmt_timestamp" => {
                    eprintln!();
                    eprintln!("💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`");
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                "esp_wifi_preempt_enable"
                | "esp_wifi_preempt_yield_task"
                | "esp_wifi_preempt_task_create" => {
                    eprintln!();
                    eprintln!("💡 `esp-wifi` has no scheduler enabled. Make sure you have the `builtin-scheduler` feature enabled, or that you provide an external scheduler.");
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!("💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests");
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=-Wl,--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}

// Image inclusion
fn include_images() {
    use png;
    use std::env;
    use std::fs;
    use std::fs::File;
    use std::io::BufReader;
    use std::path::Path;

    let out_dir = env::var("OUT_DIR").unwrap();
    let out_path = Path::new(&out_dir);

    // Create res directory path
    let res_dir = Path::new("res");
    println!("cargo:rerun-if-changed=res/");

    // Check if res directory exists
    if !res_dir.exists() {
        eprintln!("res/ directory not found");
        std::process::exit(1);
    }

    // Look for files 0.png, 1.png, 2.png, etc.
    let mut patterns_array_code = String::new();
    let mut i = 0;
    loop {
        let png_path = res_dir.join(format!("{}.png", i));
        // Intentionally print one more file than actually exists,
        // as this will cause a rerun if more files are added
        println!("cargo:rerun-if-changed={}", png_path.to_string_lossy());
        if !png_path.exists() {
            break;
        }

        // Decode the PNG
        let decoder = png::Decoder::new(BufReader::new(File::open(png_path).unwrap()));
        let mut reader = decoder.read_info().unwrap();

        // Allocate buffer for the image data
        let mut buf = vec![0; reader.output_buffer_size().unwrap()];

        // Read the image data
        let info = reader.next_frame(&mut buf).unwrap();

        // Check image dimensions and format info
        const HEIGHT: u32 = 40;
        assert_eq!(info.height, HEIGHT);
        assert_eq!(info.bit_depth, png::BitDepth::Eight);

        let buf = &buf;

        let bytes: Vec<u8> = match info.color_type {
            png::ColorType::Rgba => (0..info.width)
                .flat_map(|x| {
                    (0..HEIGHT).flat_map(move |y| {
                        let ix = (4 * (y * info.width + x)) as usize;
                        let r = buf[ix];
                        let g = buf[ix + 1];
                        let b = buf[ix + 2];
                        let a = buf[ix + 3];
                        [r * a / 255, g * a / 255, b * a / 255]
                    })
                })
                .collect(),
            png::ColorType::Rgb => (0..info.width)
                .flat_map(|x| {
                    (0..HEIGHT).flat_map(move |y| {
                        let ix = (3 * (y * info.width + x)) as usize;
                        let r = buf[ix];
                        let g = buf[ix + 1];
                        let b = buf[ix + 2];
                        [r, g, b]
                    })
                })
                .collect(),
            _ => panic!("Unsupported color type: {:?}", info.color_type),
        };

        // Write output file
        let raw_path = out_path.join(format!("{}.raw", i));
        fs::write(&raw_path, bytes).unwrap();

        // Add to PATTERNS array
        patterns_array_code.push_str(&format!(
            "    PatternData {{ texture: include_bytes!({:?}) }},\n",
            &raw_path
        ));

        i += 1;
    }

    // Write rust code for PATTERNS array
    let raw_path = out_path.join("patterns.rs");
    let rust_code = format!(
        "const PATTERNS: [PatternData; {}] = [\n{}];\n",
        i, patterns_array_code
    );
    fs::write(&raw_path, rust_code).unwrap();
}
