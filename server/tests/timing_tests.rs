//! Timing tests for benchmarking function execution.
//! These tests are designed to measure and record timing information
//! for comparison with other systems.

#[cfg(all(feature = "kvm", feature = "reqwest_io"))]
mod timing_tests {

    use assert_cmd::prelude::*;
    use dandelion_server::{DandelionDeserializeResponse, DandelionRequest, InputItem, InputSet};
    use reqwest::blocking::Client;
    use serde::Serialize;
    use serial_test::serial;
    use std::{
        fs::File,
        io::{BufRead, BufReader, Read, Write},
        process::{Child, Command, Stdio},
    };

    struct ServerKiller {
        server: Child,
    }

    #[derive(Serialize)]
    struct RegisterFunctionLocal {
        name: String,
        context_size: u64,
        engine_type: String,
        local_path: String,
        binary: Vec<u8>,
        input_sets: Vec<(String, Option<Vec<(String, Vec<u8>)>>)>,
        output_sets: Vec<String>,
    }

    #[derive(Serialize)]
    struct RegisterChain {
        composition: String,
    }

    impl Drop for ServerKiller {
        fn drop(&mut self) {
            let mut kill = Command::new("kill")
                .stdout(Stdio::piped())
                .args(["-s", "TERM", &self.server.id().to_string()])
                .spawn()
                .unwrap();
            kill.wait().unwrap();

            if let Some(mut child_stdout) = self.server.stdout.take() {
                let mut outbuf = Vec::new();
                let _ = child_stdout
                    .read_to_end(&mut outbuf)
                    .expect("should be able to read child output after killing it");
                print!(
                    "server output:\n{}",
                    String::from_utf8(outbuf)
                        .expect("Should be able to convert child stdout to string")
                );
            }
            let mut errbuf = Vec::new();
            let _ = self
                .server
                .stderr
                .take()
                .expect("Should have stderr pipe for child")
                .read_to_end(&mut errbuf)
                .expect("Should be able to read child stderr");
            print!(
                "server stderr:\n{}",
                String::from_utf8(errbuf).expect("Server stderr should be string")
            )
        }
    }

    fn start_server() -> ServerKiller {
        let mut cmd = Command::cargo_bin("dandelion_server").unwrap();
        let server = cmd
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let mut server_killer = ServerKiller { server };
        let mut reader = BufReader::new(server_killer.server.stdout.take().unwrap());
        loop {
            let mut buf = String::new();
            let len = reader.read_line(&mut buf).unwrap();
            assert_ne!(len, 0, "Server exited unexpectedly");
            if buf.contains("Server start") {
                break;
            } else {
                print!("{}", buf);
            }
        }
        let _ = server_killer.server.stdout.insert(reader.into_inner());
        server_killer
    }

    fn fetch_and_save_stats(client: &Client, output_path: &str) {
        let stats_resp = client
            .get("http://localhost:8080/stats")
            .send()
            .expect("Should be able to fetch stats");

        let stats_body = stats_resp.text().expect("Stats should be text");

        let mut file = File::create(output_path).expect("Should be able to create stats file");
        file.write_all(stats_body.as_bytes())
            .expect("Should be able to write stats");

        println!("Timing stats written to {}", output_path);
        println!("Stats content:\n{}", stats_body);
    }

    /// Test configuration for different function types
    #[derive(Clone)]
    struct TestConfig {
        /// Name to register the function under
        name: String,
        /// Binary filename (without path prefix)
        binary_name: String,
        /// Context size for the function
        context_size: u64,
        /// Input set names (empty for basic test)
        input_sets: Vec<String>,
        /// Output set names (empty for basic test)
        output_sets: Vec<String>,
    }

    impl TestConfig {
        fn basic() -> Self {
            Self {
                name: String::from("basic"),
                binary_name: format!("test_elf_kvm_{}_basic", std::env::consts::ARCH),
                context_size: 0x200_0000, // 32 MiB
                input_sets: vec![],
                output_sets: vec![],
            }
        }

        fn matmul() -> Self {
            Self {
                name: String::from("matmul"),
                binary_name: format!("test_elf_kvm_{}_matmul", std::env::consts::ARCH),
                context_size: 0x802_0000,
                input_sets: vec![String::from("")],
                output_sets: vec![String::from("")],
            }
        }

        fn get_binary_path(&self) -> String {
            format!(
                "{}/../machine_interface/tests/data/{}",
                env!("CARGO_MANIFEST_DIR"),
                self.binary_name,
            )
        }
    }

    fn register_function(client: &Client, config: &TestConfig) {
        let register_request = bson::to_vec(&RegisterFunctionLocal {
            name: config.name.clone(),
            context_size: config.context_size,
            local_path: config.get_binary_path(),
            binary: Vec::new(),
            engine_type: String::from("Kvm"),
            input_sets: config
                .input_sets
                .iter()
                .map(|s| (s.clone(), None))
                .collect(),
            output_sets: config.output_sets.clone(),
        })
        .unwrap();

        let registration_resp = client
            .post("http://localhost:8080/register/function")
            .body(register_request)
            .send()
            .unwrap();

        assert!(
            registration_resp.status().is_success(),
            "Function registration failed: {:?}",
            registration_resp.text()
        );
    }

    fn register_composition(client: &Client, config: &TestConfig) {
        let composition_name = format!("{}_composition", config.name);

        // Build input/output mapping for composition
        let input_mapping = if config.input_sets.is_empty() {
            String::new()
        } else {
            format!(
                "({} = all CompIn)",
                config.input_sets.first().unwrap_or(&String::from("In"))
            )
        };

        let output_mapping = if config.output_sets.is_empty() {
            String::new()
        } else {
            format!(
                "(CompOut = {})",
                config.output_sets.first().unwrap_or(&String::from("Out"))
            )
        };

        let composition = if config.input_sets.is_empty() && config.output_sets.is_empty() {
            // Basic function with no inputs/outputs
            format!(
                r#"
                function {function} () => ();
                composition {comp} () => () {{
                    {function} () => ();
                }}
                "#,
                function = config.name,
                comp = composition_name,
            )
        } else {
            // Function with inputs and outputs
            format!(
                r#"
                function {function} (In) => (Out);
                composition {comp} (CompIn) => (CompOut) {{
                    {function} {input} => {output};
                }}
                "#,
                function = config.name,
                comp = composition_name,
                input = input_mapping,
                output = output_mapping,
            )
        };

        let chain_request = RegisterChain { composition };

        let chain_resp = client
            .post("http://localhost:8080/register/composition")
            .body(bson::to_vec(&chain_request).unwrap())
            .send()
            .unwrap();

        assert!(
            chain_resp.status().is_success(),
            "Composition registration failed: {:?}",
            chain_resp.text()
        );
    }

    fn execute_function(client: &Client, config: &TestConfig) {
        let composition_name = format!("{}_composition", config.name);

        // Build request data outside the if block so it lives long enough
        let mut data = Vec::new();
        if !config.input_sets.is_empty() {
            data.extend_from_slice(&i64::to_le_bytes(1)); // matrix size
            data.extend_from_slice(&i64::to_le_bytes(1)); // value
        }

        let request = if config.input_sets.is_empty() {
            // Basic function with no inputs
            DandelionRequest {
                name: composition_name,
                sets: vec![],
            }
        } else {
            // Matmul-style function with input data
            DandelionRequest {
                name: composition_name,
                sets: vec![InputSet {
                    identifier: String::from(""),
                    items: vec![InputItem {
                        identifier: String::from(""),
                        key: 0,
                        data: &data,
                    }],
                }],
            }
        };

        let resp = client
            .post("http://localhost:8080/hot/matmul")
            .body(bson::to_vec(&request).unwrap())
            .send()
            .unwrap();

        assert!(resp.status().is_success(), "Function execution failed");

        // Verify response if there are expected outputs
        if !config.output_sets.is_empty() {
            let body = resp.bytes().unwrap();
            let response: DandelionDeserializeResponse = bson::from_slice(&body).unwrap();
            assert!(!response.sets.is_empty(), "Expected output sets");
        }
    }

    fn run_timing_test(config: TestConfig, stats_output: &str) {
        let mut server_killer = start_server();

        let client = reqwest::blocking::Client::builder()
            .http2_prior_knowledge()
            .build()
            .unwrap();

        // Register and execute function
        register_function(&client, &config);
        register_composition(&client, &config);
        execute_function(&client, &config);

        // Fetch and save timing stats
        fetch_and_save_stats(&client, stats_output);

        // Verify server didn't crash
        let status_result = server_killer.server.try_wait();
        drop(server_killer);
        let status = status_result.unwrap();
        assert_eq!(status, None, "Server exited unexpectedly");
    }

    #[test]
    #[serial]
    fn timing_basic() {
        run_timing_test(TestConfig::basic(), "stats_basic.log");
    }

    #[test]
    #[serial]
    fn timing_matmul() {
        run_timing_test(TestConfig::matmul(), "stats_matmul.log");
    }

    /// Run the basic test multiple times and collect timing data
    #[test]
    #[serial]
    fn timing_basic_repeated() {
        let mut server_killer = start_server();

        let client = reqwest::blocking::Client::builder()
            .http2_prior_knowledge()
            .build()
            .unwrap();

        let config = TestConfig::basic();

        // Register function and composition once
        register_function(&client, &config);
        register_composition(&client, &config);

        // Execute multiple times (hot path)
        const ITERATIONS: usize = 5;
        for i in 0..ITERATIONS {
            println!("Iteration {}/{}", i + 1, ITERATIONS);
            execute_function(&client, &config);
        }

        // Fetch and save all timing stats
        fetch_and_save_stats(&client, "stats_basic_repeated.log");

        let status_result = server_killer.server.try_wait();
        drop(server_killer);
        let status = status_result.unwrap();
        assert_eq!(status, None, "Server exited unexpectedly");
    }
}
